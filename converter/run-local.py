#!/usr/bin/python3 -u
"""
run-local.py: Run the Touch Mapper conversion pipeline locally.

Replaces process-request.py for local use — no SQS, no S3, no boto3.
Called by server.js for every map creation request.

Required environment (same as the full pipeline):
  - Java (for OSM2World)
  - Blender 2.78 at converter/blender/blender[.exe]
    or TOUCH_MAPPER_BLENDER_PATH env var pointing to the executable
  - Node.js (for clip-2d.js)
  - Python packages: none beyond stdlib (map_desc uses only stdlib too)
"""

import sys
import os
import json
import argparse
import urllib.request
import random
import re
import copy
import shutil
import subprocess
import math
import zlib
import struct

script_dir = os.path.dirname(os.path.realpath(__file__))

MAX_OSM_BYTES = 25 * 1024 * 1024

# NRW LOD2 open-data tile download (1 km × 1 km CityGML tiles, UTM32N tile index)
_NRW_LOD2_TILE_BASE = 'https://www.opengeodata.nrw.de/produkte/geobasis/3dg/lod2_gml/lod2_gml/'
_NRW_LON_MIN, _NRW_LON_MAX = 5.8, 9.5
_NRW_LAT_MIN, _NRW_LAT_MAX = 50.3, 52.5


def write_status(info_path, request_id, progress, error_code=None, error_desc=None):
    status_obj = {'progress': progress}
    if error_code:
        status_obj['errorCode'] = error_code
        status_obj['errorDescription'] = error_desc or error_code
    payload = {'requestId': request_id, 'status': status_obj}
    with open(info_path, 'w', encoding='utf-8') as f:
        json.dump(payload, f)


def add_or_replace_bounds(osm_text, eff_area):
    bounds_line = '  <bounds minlat="{}" minlon="{}" maxlat="{}" maxlon="{}"/>'.format(
        eff_area['latMin'], eff_area['lonMin'],
        eff_area['latMax'], eff_area['lonMax'],
    )
    if re.search(r'<bounds [^>]+/>\s*', osm_text):
        return re.sub(r'<bounds [^>]+/>\s*', bounds_line + '\n', osm_text, count=1)
    if re.search(r'<meta [^>]+/>\s*', osm_text):
        return re.sub(r'(<meta [^>]+/>\s*)', r'\1' + bounds_line + '\n', osm_text, count=1)
    return re.sub(r'(<osm[^>]*>\s*)', r'\1' + bounds_line + '\n', osm_text, count=1)


def _decode_png_rgb(data):
    """Decode PNG bytes to (width, height, [(R,G,B),...]) using stdlib only."""
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('not a PNG file')
    pos = 8
    width = height = bit_depth = color_type = 0
    idat = []
    while pos < len(data):
        length = struct.unpack('>I', data[pos:pos+4])[0]
        ctype = data[pos+4:pos+8]
        cdata = data[pos+8:pos+8+length]
        pos += 12 + length
        if ctype == b'IHDR':
            width, height = struct.unpack('>II', cdata[:8])
            bit_depth, color_type = cdata[8], cdata[9]
        elif ctype == b'IDAT':
            idat.append(cdata)
        elif ctype == b'IEND':
            break
    if bit_depth != 8:
        raise ValueError('unsupported PNG bit depth: {}'.format(bit_depth))
    bpp = {2: 3, 6: 4}.get(color_type)
    if bpp is None:
        raise ValueError('unsupported PNG color type: {}'.format(color_type))
    raw = zlib.decompress(b''.join(idat))
    stride = 1 + width * bpp
    prev = bytearray(width * bpp)
    pixels = []
    for y in range(height):
        ftype = raw[y * stride]
        row = bytearray(raw[y * stride + 1:(y + 1) * stride])
        if ftype == 1:    # Sub
            for i in range(bpp, len(row)):
                row[i] = (row[i] + row[i - bpp]) & 0xFF
        elif ftype == 2:  # Up
            for i in range(len(row)):
                row[i] = (row[i] + prev[i]) & 0xFF
        elif ftype == 3:  # Average
            for i in range(len(row)):
                a = row[i - bpp] if i >= bpp else 0
                row[i] = (row[i] + (a + prev[i]) // 2) & 0xFF
        elif ftype == 4:  # Paeth
            for i in range(len(row)):
                a = row[i - bpp] if i >= bpp else 0
                b = prev[i]
                c = prev[i - bpp] if i >= bpp else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                row[i] = (row[i] + pr) & 0xFF
        for x in range(width):
            pixels.append((row[x * bpp], row[x * bpp + 1], row[x * bpp + 2]))
        prev = row
    return width, height, pixels


def _fetch_terrain_rgb_elevations(eff_area, grid_size=40, zoom=15):
    """Sample elevation from AWS Terrain-RGB (Terrarium) tiles. No API key required.
    Terrarium encoding: elevation = R*256 + G + B/256 - 32768 (metres).
    """
    lat_min, lat_max = eff_area['latMin'], eff_area['latMax']
    lon_min, lon_max = eff_area['lonMin'], eff_area['lonMax']

    def to_tile_f(lat, lon):
        n = 1 << zoom
        tx = (lon + 180.0) / 360.0 * n
        lr = math.radians(lat)
        ty = (1.0 - math.log(math.tan(lr) + 1.0 / math.cos(lr)) / math.pi) / 2.0 * n
        return tx, ty

    # Collect ALL tiles in the bounding box (corners only misses middle tiles)
    itx_list, ity_list = [], []
    for lat in [lat_min, lat_max]:
        for lon in [lon_min, lon_max]:
            tx, ty = to_tile_f(lat, lon)
            itx_list.append(int(tx))
            ity_list.append(int(ty))
    tile_set = set()
    for itx in range(min(itx_list), max(itx_list) + 1):
        for ity in range(min(ity_list), max(ity_list) + 1):
            tile_set.add((itx, ity))

    # Fetch and decode tiles
    tile_cache = {}
    for itx, ity in sorted(tile_set):
        url = 'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{}/{}/{}.png'.format(zoom, itx, ity)
        print('  terrain tile z={} x={} y={}'.format(zoom, itx, ity), flush=True)
        req = urllib.request.Request(url, headers={'User-Agent': 'touch-mapper/1.0'})
        resp = urllib.request.urlopen(req, timeout=30)
        w, h, pixels = _decode_png_rgb(resp.read())
        tile_cache[(itx, ity)] = (w, h, pixels)

    def sample(lat, lon):
        tx, ty = to_tile_f(lat, lon)
        itx, ity = int(tx), int(ty)
        entry = tile_cache.get((itx, ity))
        if entry is None:
            return 0.0
        w, h, pixels = entry
        px = (tx - itx) * w
        py = (ty - ity) * h
        x0 = max(0, min(w - 2, int(px)))
        y0 = max(0, min(h - 2, int(py)))
        x1, y1 = x0 + 1, y0 + 1
        fx, fy = px - x0, py - y0
        def elev(x, y):
            r, g, b = pixels[y * w + x]
            return r * 256.0 + g + b / 256.0 - 32768.0
        return (elev(x0, y0)*(1-fx)*(1-fy) + elev(x1, y0)*fx*(1-fy) +
                elev(x0, y1)*(1-fx)*fy     + elev(x1, y1)*fx*fy)

    lats = [lat_min + (lat_max - lat_min) * i / (grid_size - 1) for i in range(grid_size)]
    lons = [lon_min + (lon_max - lon_min) * j / (grid_size - 1) for j in range(grid_size)]
    return [[sample(lat, lon) for lon in lons] for lat in lats]


def _box_blur_once(grid, grid_size):
    """One 3×3 average pass; edges keep their value."""
    out = [row[:] for row in grid]
    for r in range(1, grid_size - 1):
        for c in range(1, grid_size - 1):
            out[r][c] = (
                grid[r-1][c-1] + grid[r-1][c] + grid[r-1][c+1] +
                grid[r  ][c-1] + grid[r  ][c] + grid[r  ][c+1] +
                grid[r+1][c-1] + grid[r+1][c] + grid[r+1][c+1]
            ) / 9.0
    return out


def _smooth_elevation_grid(elevations, grid_size, amount=2.0):
    """Smooth the elevation grid by a fractional amount (0 = off).

    Each whole unit of `amount` is one full 3×3 box-blur pass; a fractional
    remainder blends partway toward the next blurred pass, so e.g. 0.2 gives a
    light 20 % smoothing and 1.5 is one full pass plus a half-strength one.
    """
    if amount <= 0:
        return elevations
    grid = [row[:] for row in elevations]
    full_passes = int(amount)
    frac = amount - full_passes
    for _ in range(full_passes):
        grid = _box_blur_once(grid, grid_size)
    if frac > 0:
        blurred = _box_blur_once(grid, grid_size)
        for r in range(grid_size):
            for c in range(grid_size):
                grid[r][c] = grid[r][c] * (1.0 - frac) + blurred[r][c] * frac
    return grid


def _fetch_elevation_batched(url, locations, batch_size=100, timeout=60):
    """POST locations to an elevation API in batches; returns flat list of results or raises."""
    results = []
    for i in range(0, len(locations), batch_size):
        batch = locations[i:i + batch_size]
        payload = json.dumps({'locations': batch}).encode('utf-8')
        req = urllib.request.Request(url, data=payload,
                                     headers={'Content-Type': 'application/json',
                                              'Accept': 'application/json'})
        resp = urllib.request.urlopen(req, timeout=timeout)
        data = json.loads(resp.read())
        batch_results = data.get('results', [])
        if len(batch_results) != len(batch):
            raise Exception('unexpected result count: {} vs {}'.format(len(batch_results), len(batch)))
        results.extend(batch_results)
    return results


# NRW DGM1: 1 m bare-earth terrain model, 1 km GeoTIFF tiles (UTM32, like the LOD2 tiles).
_DGM1_TILE_BASE = 'https://www.opengeodata.nrw.de/produkte/geobasis/hm/dgm1_tiff/dgm1_tiff/'
_DGM1_TARGET_CELL_M = 1.0   # aim for ~1 m terrain cells (DGM1 native resolution)
_DGM1_MIN_GRID = 80
_DGM1_MAX_GRID = 320        # cap polygon count on large maps (320x320 grid)


def _adaptive_dgm1_grid(eff_area):
    """Grid resolution sized to ~1 m/cell for the map, clamped to keep polys sane."""
    mid_lat = (eff_area['latMin'] + eff_area['latMax']) / 2.0
    width_m = abs(eff_area['lonMax'] - eff_area['lonMin']) * 111320.0 * math.cos(math.radians(mid_lat))
    height_m = abs(eff_area['latMax'] - eff_area['latMin']) * 111320.0
    size_m = max(width_m, height_m)
    grid = int(size_m / _DGM1_TARGET_CELL_M) + 1
    return max(_DGM1_MIN_GRID, min(grid, _DGM1_MAX_GRID))


def _dgm1_load_index(cache_dir):
    """Download/cache the DGM1 tile index, return dict (tileX, tileY) -> filename."""
    idx_path = os.path.join(cache_dir, 'dgm1_index.json')
    if not os.path.exists(idx_path):
        print('DGM1: downloading tile index ...', flush=True)
        tmp = idx_path + '.tmp'
        urllib.request.urlretrieve(_DGM1_TILE_BASE + 'index.json', tmp)
        os.replace(tmp, idx_path)
    with open(idx_path, 'r', encoding='utf-8') as f:
        idx = json.load(f)
    mapping = {}
    for ds in idx.get('datasets', []):
        for fobj in ds.get('files', []):
            name = fobj.get('name', '')
            m = re.match(r'dgm1_32_(\d+)_(\d+)_1_nw', name)
            if m:
                mapping[(int(m.group(1)), int(m.group(2)))] = name
    return mapping


def _fetch_dgm1_elevation(eff_area, grid_size):
    """Sample a grid_size x grid_size elevation grid from NRW DGM1 1 m tiles.

    Returns the grid (list of rows) or None if the area is outside NRW / unavailable,
    so the caller can fall back to the global elevation sources.
    """
    lon_min = eff_area['lonMin']; lon_max = eff_area['lonMax']
    lat_min = eff_area['latMin']; lat_max = eff_area['latMax']
    if not (lon_min < _NRW_LON_MAX and lon_max > _NRW_LON_MIN and
            lat_min < _NRW_LAT_MAX and lat_max > _NRW_LAT_MIN):
        return None  # outside NRW

    try:
        import numpy as np
        from PIL import Image
    except Exception as e:
        print('DGM1: numpy/PIL unavailable ({}), using fallback'.format(e), flush=True)
        return None

    cache_dir = os.path.join(script_dir, '..', 'local-data', 'dgm1-cache')
    os.makedirs(cache_dir, exist_ok=True)
    try:
        index = _dgm1_load_index(cache_dir)
    except Exception as e:
        print('DGM1: index unavailable ({}), using fallback'.format(e), flush=True)
        return None

    tiles = {}

    def get_tile(tx, ty):
        if (tx, ty) in tiles:
            return tiles[(tx, ty)]
        arr = None
        name = index.get((tx, ty))
        if name:
            path = os.path.join(cache_dir, name)
            if not os.path.exists(path):
                print('DGM1: downloading {} ...'.format(name), flush=True)
                try:
                    tmp = path + '.tmp'
                    urllib.request.urlretrieve(_DGM1_TILE_BASE + name, tmp)
                    os.replace(tmp, path)
                except Exception as e:
                    print('DGM1: download failed ({}): {}'.format(name, e), flush=True)
            if os.path.exists(path):
                try:
                    arr = np.asarray(Image.open(path), dtype='float32')
                except Exception as e:
                    print('DGM1: read failed ({}): {}'.format(name, e), flush=True)
        tiles[(tx, ty)] = arr
        return arr

    lats = [lat_min + (lat_max - lat_min) * i / (grid_size - 1) for i in range(grid_size)]
    lons = [lon_min + (lon_max - lon_min) * j / (grid_size - 1) for j in range(grid_size)]

    elevations = []
    missing = 0
    for i in range(grid_size):
        row = []
        for j in range(grid_size):
            e, n = _wgs84_to_utm32n(lons[j], lats[i])
            tx = int(e // 1000); ty = int(n // 1000)
            arr = get_tile(tx, ty)
            value = None
            if arr is not None:
                h, w = arr.shape
                fx = e - tx * 1000.0              # column, west->east
                fy = (ty * 1000.0 + 1000.0) - n   # row, north(top)->south
                fx = min(max(fx, 0.0), w - 1.0)
                fy = min(max(fy, 0.0), h - 1.0)
                x0 = int(fx); y0 = int(fy)
                x1 = min(x0 + 1, w - 1); y1 = min(y0 + 1, h - 1)
                tX = fx - x0; tY = fy - y0
                v = (arr[y0, x0] * (1 - tX) * (1 - tY) + arr[y0, x1] * tX * (1 - tY) +
                     arr[y1, x0] * (1 - tX) * tY + arr[y1, x1] * tX * tY)
                v = float(v)
                if -1000.0 < v < 9000.0:  # reject DGM1 nodata (-9999) / garbage
                    value = v
            if value is None:
                missing += 1
            row.append(value)
        elevations.append(row)

    valid = [v for r in elevations for v in r if v is not None]
    if not valid or missing > grid_size * grid_size * 0.5:
        print('DGM1: {} of {} samples missing — using fallback'.format(
            missing, grid_size * grid_size), flush=True)
        return None
    mean = sum(valid) / len(valid)
    for r in range(grid_size):
        for c in range(grid_size):
            if elevations[r][c] is None:
                elevations[r][c] = mean
    n_tiles = sum(1 for v in tiles.values() if v is not None)
    print('DGM1: {}x{} grid from {} tile(s), {} samples filled'.format(
        grid_size, grid_size, n_tiles, missing), flush=True)
    return elevations


def fetch_elevation(eff_area, work_dir, grid_size=80, smoothing_amount=2.0):
    """Fetch elevation grid; primary (NRW) = DGM1 1 m, then Terrain-RGB tiles, then APIs.

    smoothing_amount controls the box-blur applied at the end (0 = no smoothing,
    keeping the full DGM1 precision; fractional values give light smoothing).
    """
    elevations = None
    used_grid = grid_size

    # Primary for NRW: DGM1 1 m bare-earth model (far better resolution than SRTM).
    try:
        dgm1_grid = _adaptive_dgm1_grid(eff_area)
        dgm1 = _fetch_dgm1_elevation(eff_area, dgm1_grid)
        if dgm1 is not None:
            elevations = dgm1
            used_grid = dgm1_grid
            print('elevation source: NRW DGM1 1 m ({}x{} grid)'.format(dgm1_grid, dgm1_grid), flush=True)
    except Exception as e:
        print('DGM1 failed: {} — trying other sources'.format(e), flush=True)
        elevations = None

    # Fallback 1: AWS Terrain-RGB (Terrarium) — free, no rate limit, no API key
    if elevations is None:
        try:
            print('fetching elevation from terrain-rgb tiles (AWS) ...', flush=True)
            elevations = _fetch_terrain_rgb_elevations(eff_area, grid_size=grid_size)
            used_grid = grid_size
            print('terrain-rgb: {}x{} grid ready'.format(grid_size, grid_size), flush=True)
        except Exception as e:
            print('terrain-rgb failed: {} — trying API fallbacks'.format(e), flush=True)

    # Fallback 2: OpenTopoData EUDEM → SRTM30m → Open-Elevation
    if elevations is None:
        used_grid = grid_size
        lats = [eff_area['latMin'] + (eff_area['latMax'] - eff_area['latMin']) * i / (grid_size - 1)
                for i in range(grid_size)]
        lons = [eff_area['lonMin'] + (eff_area['lonMax'] - eff_area['lonMin']) * j / (grid_size - 1)
                for j in range(grid_size)]
        locations = [{'latitude': lats[i], 'longitude': lons[j]}
                     for i in range(grid_size) for j in range(grid_size)]
        apis = [
            ('https://api.opentopodata.org/v1/eudem25m',     'eudem25m',      100),
            ('https://api.opentopodata.org/v1/srtm30m',      'srtm30m',       100),
            ('https://api.open-elevation.com/api/v1/lookup', 'open-elevation', 512),
        ]
        last_err = None
        results = None
        for url, name, batch_size in apis:
            try:
                print('fetching elevation from {} ...'.format(name), flush=True)
                results = _fetch_elevation_batched(url, locations, batch_size=batch_size)
                if len(results) == grid_size * grid_size:
                    print('elevation fetched from {}: {} points'.format(name, len(results)), flush=True)
                    break
                results = None
            except Exception as e:
                last_err = e
                print('elevation API {} failed: {}'.format(name, e), flush=True)
        if results:
            elevations = [[results[i * grid_size + j]['elevation']
                           for j in range(grid_size)]
                          for i in range(grid_size)]
        else:
            raise Exception('All elevation sources failed. Last: ' + str(last_err))

    if smoothing_amount and smoothing_amount > 0:
        elevations = _smooth_elevation_grid(elevations, used_grid, amount=smoothing_amount)
        print('elevation smoothed: amount {}'.format(smoothing_amount), flush=True)
    else:
        print('elevation smoothing disabled (full precision)', flush=True)

    elev_path = os.path.join(work_dir, 'elevation.json')
    with open(elev_path, 'w', encoding='utf-8') as f:
        json.dump({'grid_size': used_grid, 'elevations': elevations}, f)
    return elev_path


def fetch_osm(eff_area, osm_path):
    bbox = '{},{},{},{}'.format(
        eff_area['lonMin'], eff_area['latMin'],
        eff_area['lonMax'], eff_area['latMax'],
    )
    overpass_urls = [
        'http://www.overpass-api.de/api/xapi?map?bbox=' + bbox,
        'http://overpass.osm.rambler.ru/cgi/xapi?map?bbox=' + bbox,
    ]
    random.shuffle(overpass_urls)
    urls = overpass_urls + ['http://api.openstreetmap.org/api/0.6/map?bbox=' + bbox]

    last_error = None
    for url in urls:
        try:
            print('fetching OSM from: ' + url, flush=True)
            response = urllib.request.urlopen(url, timeout=120)
            data = response.read()
            if len(data) > MAX_OSM_BYTES:
                raise Exception('OSM data too large: {} bytes'.format(len(data)))
            text = add_or_replace_bounds(data.decode('utf-8'), eff_area)
            with open(osm_path, 'w', encoding='utf-8') as f:
                f.write(text)
            print('OSM fetched: {} bytes'.format(len(data)), flush=True)
            return
        except Exception as e:
            last_error = e
            print('failed ({}): {}'.format(url, e), flush=True)

    raise Exception('All OSM fetch attempts failed. Last error: ' + str(last_error))


def _extract_building_heights(osm_path, eff_area, work_dir):
    """Parse OSM file for building height/level tags, write building-heights.json."""
    import xml.etree.ElementTree as ET
    try:
        tree = ET.parse(osm_path)
    except Exception as e:
        print('building heights: OSM parse failed: {}'.format(e), flush=True)
        return None

    root = tree.getroot()
    nodes = {}
    for node in root.findall('node'):
        nid = node.get('id')
        lat = node.get('lat')
        lon = node.get('lon')
        if nid and lat and lon:
            nodes[nid] = (float(lat), float(lon))

    buildings = []
    for way in root.findall('way'):
        tags = {t.get('k'): t.get('v') for t in way.findall('tag')}
        if 'building' not in tags and 'building:part' not in tags:
            continue
        height_m = None
        if 'height' in tags:
            try:
                height_m = float(tags['height'].rstrip('m').strip())
            except ValueError:
                pass
        if height_m is None and 'building:levels' in tags:
            try:
                height_m = float(tags['building:levels']) * 3.0
            except ValueError:
                pass
        if height_m is None:
            continue

        nds = [nodes[nd.get('ref')] for nd in way.findall('nd') if nd.get('ref') in nodes]
        if not nds:
            continue
        lat_c = sum(n[0] for n in nds) / len(nds)
        lon_c = sum(n[1] for n in nds) / len(nds)
        buildings.append({'lat': lat_c, 'lon': lon_c, 'height_m': height_m})

    if not buildings:
        print('building heights: no buildings with height data found', flush=True)
        return None

    bh_path = os.path.join(work_dir, 'building-heights.json')
    with open(bh_path, 'w', encoding='utf-8') as f:
        json.dump({
            'lon_min': eff_area['lonMin'], 'lon_max': eff_area['lonMax'],
            'lat_min': eff_area['latMin'], 'lat_max': eff_area['latMax'],
            'buildings': buildings,
        }, f)
    print('building heights: {} buildings with height data'.format(len(buildings)), flush=True)
    return bh_path


def _extract_roof_shapes(osm_path, eff_area, work_dir):
    """Parse OSM for per-building roof shapes using osm_roof_fixer classification logic."""
    import xml.etree.ElementTree as ET
    import math as _math

    _FLAT = {'industrial', 'warehouse', 'supermarket', 'retail', 'commercial',
             'garage', 'garages', 'carport', 'train_station', 'transportation',
             'parking', 'hangar', 'storage_tank'}
    _GABLED = {'house', 'detached', 'semidetached_house', 'terrace', 'bungalow',
               'cabin', 'farm', 'farm_auxiliary', 'barn', 'stable', 'shed', 'greenhouse'}
    _PYRAMIDAL = {'church', 'chapel', 'cathedral', 'temple', 'mosque', 'synagogue'}

    def _classify(tags, coords):
        existing = tags.get('roof:shape')
        if existing:
            return existing
        btype = tags.get('building', tags.get('building:part', 'yes'))
        if btype in _FLAT:
            return 'flat'
        if btype in _PYRAMIDAL:
            return 'pyramidal'
        if btype in _GABLED:
            return 'gabled'
        if coords and len(coords) >= 3:
            lats = [c[0] for c in coords]
            lons = [c[1] for c in coords]
            dlat = (max(lats) - min(lats)) * 111320
            dlon = (max(lons) - min(lons)) * 111320 * _math.cos(_math.radians(sum(lats) / len(lats)))
            area = dlat * max(dlon, 0.01)
            if area > 1500:
                return 'flat'
            ratio = max(dlat, dlon) / max(min(dlat, dlon), 0.01)
            if ratio > 2.5:
                return 'gabled'
            if ratio < 1.5:
                return 'hipped'
        return 'gabled'

    try:
        tree = ET.parse(osm_path)
    except Exception as e:
        print('roof shapes: OSM parse failed: {}'.format(e), flush=True)
        return None

    root = tree.getroot()
    node_map = {}
    for node in root.findall('node'):
        nid = node.get('id')
        lat = node.get('lat')
        lon = node.get('lon')
        if nid and lat and lon:
            node_map[nid] = (float(lat), float(lon))

    buildings = []
    for way in root.findall('way'):
        tags = {t.get('k'): t.get('v') for t in way.findall('tag')}
        if 'building' not in tags and 'building:part' not in tags:
            continue
        nds = [node_map[nd.get('ref')] for nd in way.findall('nd') if nd.get('ref') in node_map]
        if not nds:
            continue
        shape = _classify(tags, nds)
        lat_c = sum(n[0] for n in nds) / len(nds)
        lon_c = sum(n[1] for n in nds) / len(nds)
        buildings.append({'lat': lat_c, 'lon': lon_c, 'roof_shape': shape})

    way_index = {w.get('id'): w for w in root.findall('way')}
    for rel in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in rel.findall('tag')}
        if 'building' not in tags and 'building:part' not in tags:
            continue
        coords = []
        for member in rel.findall('member'):
            if member.get('type') == 'way' and member.get('role') == 'outer':
                w = way_index.get(member.get('ref'))
                if w is not None:
                    coords.extend(node_map[nd.get('ref')] for nd in w.findall('nd') if nd.get('ref') in node_map)
        if not coords:
            continue
        shape = _classify(tags, coords)
        lat_c = sum(c[0] for c in coords) / len(coords)
        lon_c = sum(c[1] for c in coords) / len(coords)
        buildings.append({'lat': lat_c, 'lon': lon_c, 'roof_shape': shape})

    if not buildings:
        print('roof shapes: no buildings found', flush=True)
        return None

    rs_path = os.path.join(work_dir, 'roof-shapes.json')
    with open(rs_path, 'w', encoding='utf-8') as f:
        json.dump({
            'lon_min': eff_area['lonMin'], 'lon_max': eff_area['lonMax'],
            'lat_min': eff_area['latMin'], 'lat_max': eff_area['latMax'],
            'buildings': buildings,
        }, f)
    shapes = {}
    for b in buildings:
        shapes[b['roof_shape']] = shapes.get(b['roof_shape'], 0) + 1
    print('roof shapes: {} buildings — {}'.format(len(buildings), shapes), flush=True)
    return rs_path


def _extract_water_towers(osm_path, eff_area, work_dir):
    """Parse OSM file for water tower nodes, write water-towers.json."""
    import xml.etree.ElementTree as ET
    try:
        tree = ET.parse(osm_path)
    except Exception as e:
        print('water towers: OSM parse failed: {}'.format(e), flush=True)
        return None

    towers = []
    for node in tree.getroot().findall('node'):
        tags = {t.get('k'): t.get('v') for t in node.findall('tag')}
        if tags.get('man_made') == 'water_tower':
            lat = node.get('lat')
            lon = node.get('lon')
            if lat and lon:
                towers.append({'lat': float(lat), 'lon': float(lon)})

    if not towers:
        print('water towers: none found', flush=True)
        return None

    wt_path = os.path.join(work_dir, 'water-towers.json')
    with open(wt_path, 'w', encoding='utf-8') as f:
        json.dump({
            'lon_min': eff_area['lonMin'], 'lon_max': eff_area['lonMax'],
            'lat_min': eff_area['latMin'], 'lat_max': eff_area['latMax'],
            'towers': towers,
        }, f)
    print('water towers: {} found'.format(len(towers)), flush=True)
    return wt_path


def _wgs84_to_utm32n(lon, lat):
    """WGS84 degrees → UTM Zone 32N easting/northing (meters). Pure Python."""
    a = 6378137.0
    e2 = 0.00669437999014
    k0 = 0.9996
    lam0 = math.radians(9.0)
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(lat_r) ** 2)
    T = math.tan(lat_r) ** 2
    C = e2 / (1 - e2) * math.cos(lat_r) ** 2
    A_ = (lon_r - lam0) * math.cos(lat_r)
    M = a * ((1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat_r
             - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat_r)
             + (15*e2**2/256 + 45*e2**3/1024) * math.sin(4*lat_r)
             - (35*e2**3/3072) * math.sin(6*lat_r))
    E = (k0 * N * (A_ + (1-T+C)*A_**3/6
         + (5-18*T+T**2+72*C-58*e2/(1-e2))*A_**5/120) + 500000.0)
    N_ = k0 * (M + N*math.tan(lat_r)*(A_**2/2
               + (5-T+9*C+4*C**2)*A_**4/24
               + (61-58*T+T**2+600*C-330*e2/(1-e2))*A_**6/720))
    return E, N_


def _utm32n_to_wgs84(easting, northing):
    """UTM Zone 32N easting/northing (meters) → WGS84 lon/lat degrees. Pure Python."""
    a = 6378137.0
    e2 = 0.00669437999014
    k0 = 0.9996
    lam0 = math.radians(9.0)
    x = easting - 500000.0
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    mu = northing / (k0 * a * (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256))
    phi1 = (mu + (3*e1/2 - 27*e1**3/32)*math.sin(2*mu)
            + (21*e1**2/16 - 55*e1**4/32)*math.sin(4*mu)
            + (151*e1**3/96)*math.sin(6*mu)
            + (1097*e1**4/512)*math.sin(8*mu))
    N1 = a / math.sqrt(1 - e2*math.sin(phi1)**2)
    T1 = math.tan(phi1)**2
    C1 = e2/(1-e2)*math.cos(phi1)**2
    R1 = a*(1-e2)/(1-e2*math.sin(phi1)**2)**1.5
    D = x / (N1*k0)
    lat = phi1 - (N1*math.tan(phi1)/R1)*(
        D**2/2 - (5+3*T1+10*C1-4*C1**2-9*e2/(1-e2))*D**4/24
        + (61+90*T1+298*C1+45*T1**2-252*e2/(1-e2)-3*C1**2)*D**6/720)
    lon = lam0 + (D - (1+2*T1+C1)*D**3/6
                  + (5-2*C1+28*T1-3*C1**2+8*e2/(1-e2)+24*T1**2)*D**5/120) / math.cos(phi1)
    return math.degrees(lon), math.degrees(lat)


def _polygon_exterior_points(poly):
    """Return the exterior LinearRing points [(easting, northing, z), ...] of a GML Polygon.

    Interior rings (courtyards/holes) are ignored so each surface becomes one
    simple face. NRW LOD2 posLists store coordinates as easting northing z.
    """
    exterior = None
    for child in poly.iter():
        if child.tag.endswith('}exterior') or child.tag == 'exterior':
            exterior = child
            break
    target = exterior if exterior is not None else poly
    for pl in target.iter():
        if not (pl.tag.endswith('}posList') or pl.tag == 'posList'):
            continue
        nums = (pl.text or '').split()
        pts = []
        i = 0
        while i + 2 < len(nums):
            try:
                pts.append((float(nums[i]), float(nums[i + 1]), float(nums[i + 2])))
            except ValueError:
                pass
            i += 3
        return pts
    return []


def _parse_lod2_gml(root):
    """Parse a CityGML tree into building dicts with full solid geometry.

    Every exterior boundary polygon of a building (GroundSurface + WallSurface +
    RoofSurface) is collected so the consumer can rebuild a closed, watertight
    solid. Coordinates are converted UTM32N -> WGS84 lon/lat; z stays in metres.
    """
    buildings = []
    for bldg in root.iter():
        if not (bldg.tag.endswith('}Building') or bldg.tag == 'Building'):
            continue
        faces_utm = []
        for poly in bldg.iter():
            if not (poly.tag.endswith('}Polygon') or poly.tag == 'Polygon'):
                continue
            pts = _polygon_exterior_points(poly)
            if len(pts) >= 3:
                faces_utm.append(pts)
        if not faces_utm:
            continue
        all_pts = [p for face in faces_utm for p in face]
        base_elev = min(p[2] for p in all_pts)
        ce = sum(p[0] for p in all_pts) / len(all_pts)
        cn = sum(p[1] for p in all_pts) / len(all_pts)
        clon, clat = _utm32n_to_wgs84(ce, cn)
        faces_wgs84 = []
        for face in faces_utm:
            flat = []
            for (e, n, z) in face:
                flon, flat_lat = _utm32n_to_wgs84(e, n)
                flat.extend([flon, flat_lat, z])
            faces_wgs84.append(flat)
        buildings.append({
            'centroid_lon': clon,
            'centroid_lat': clat,
            'base_elev': base_elev,
            'faces': faces_wgs84,
        })
    return buildings


def _parse_lod2_gml_file(path):
    """Stream-parse a CityGML file using iterparse to keep memory low. Returns building list."""
    import xml.etree.ElementTree as ET
    buildings = []
    # We accumulate a complete Building element, then hand it to _parse_lod2_gml.
    # iterparse fires 'end' after all children of an element are parsed, so we
    # can process the Building sub-tree and then clear it to reclaim memory.
    in_building = False
    bldg_elem = None
    for event, elem in ET.iterparse(path, events=('start', 'end')):
        is_bldg_tag = elem.tag.endswith('}Building') or elem.tag == 'Building'
        if event == 'start':
            if is_bldg_tag and not in_building:
                in_building = True
                bldg_elem = elem
        elif event == 'end':
            if is_bldg_tag and in_building and elem is bldg_elem:
                parsed = _parse_lod2_gml(elem)
                buildings.extend(parsed)
                in_building = False
                bldg_elem = None
                elem.clear()
    return buildings


def _fetch_lod2_roofs(eff_area, work_dir):
    """Download NRW LOD2 CityGML tiles for the map bbox. Returns JSON path or None.

    Tiles come from the NRW open-data portal (1 km × 1 km, UTM32N tile index).
    Downloaded tiles are cached in local-data/lod2-cache/ next to the repo root.
    """
    lon_min = eff_area['lonMin']
    lon_max = eff_area['lonMax']
    lat_min = eff_area['latMin']
    lat_max = eff_area['latMax']

    if not (lon_min < _NRW_LON_MAX and lon_max > _NRW_LON_MIN and
            lat_min < _NRW_LAT_MAX and lat_max > _NRW_LAT_MIN):
        return None  # outside NRW

    buf = 0.001
    e_min, n_min = _wgs84_to_utm32n(lon_min - buf, lat_min - buf)
    e_max, n_max = _wgs84_to_utm32n(lon_max + buf, lat_max + buf)

    # Tile index: tile X_Y covers easting [X*1000, (X+1)*1000) and northing [Y*1000, (Y+1)*1000)
    x_min = int(e_min // 1000)
    x_max = int(e_max // 1000)
    y_min = int(n_min // 1000)
    y_max = int(n_max // 1000)

    cache_dir = os.path.join(script_dir, '..', 'local-data', 'lod2-cache')
    os.makedirs(cache_dir, exist_ok=True)

    all_buildings = []
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            filename = 'LoD2_32_{:d}_{:d}_1_NW.gml'.format(x, y)
            cache_path = os.path.join(cache_dir, filename)

            if not os.path.exists(cache_path):
                url = _NRW_LOD2_TILE_BASE + filename
                print('LOD2: downloading {} ...'.format(filename), flush=True)
                try:
                    with urllib.request.urlopen(url, timeout=120) as r:
                        data = r.read()
                    tmp = cache_path + '.tmp'
                    with open(tmp, 'wb') as f:
                        f.write(data)
                    os.replace(tmp, cache_path)
                    print('LOD2: saved {:,.0f} kB'.format(len(data) / 1024), flush=True)
                except Exception as e:
                    print('LOD2: download failed ({}): {}'.format(filename, e), flush=True)
                    tmp = cache_path + '.tmp'
                    if os.path.exists(tmp):
                        os.remove(tmp)
                    continue

            print('LOD2: parsing {} ...'.format(filename), flush=True)
            try:
                buildings = _parse_lod2_gml_file(cache_path)
            except Exception as e:
                print('LOD2: parse failed ({}): {}'.format(filename, e), flush=True)
                continue

            for b in buildings:
                clon = b['centroid_lon']
                clat = b['centroid_lat']
                if (lon_min - buf <= clon <= lon_max + buf and
                        lat_min - buf <= clat <= lat_max + buf):
                    all_buildings.append(b)

    if not all_buildings:
        print('LOD2: no buildings found in area', flush=True)
        return None

    out_path = os.path.join(work_dir, 'lod2-buildings.json')
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({
            'lon_min': lon_min, 'lon_max': lon_max,
            'lat_min': lat_min, 'lat_max': lat_max,
            'buildings': all_buildings,
        }, f, separators=(',', ':'))
    print('LOD2: {} buildings written'.format(len(all_buildings)), flush=True)
    return out_path


def run_osm_to_tactile(work_dir, request_body):
    eff_area = request_body['effectiveArea']
    osm_path = os.path.join(work_dir, 'map.osm')

    args = [
        sys.executable, os.path.join(script_dir, 'osm-to-tactile.py'),
        '--scale', str(request_body['scale']),
        '--diameter', str(request_body['diameter']),
        '--size', str(request_body['size']),
    ]

    if not request_body.get('noBorders', True):
        pass  # no-borders is the default; only skip it when withBorders is explicitly set
    else:
        args.append('--no-borders')

    if request_body.get('realBuildingHeights'):
        bh_path = _extract_building_heights(osm_path, eff_area, work_dir)
        if bh_path:
            args.extend(['--building-heights-json', bh_path])

    rs_path = _extract_roof_shapes(osm_path, eff_area, work_dir)
    if rs_path:
        args.extend(['--roof-shapes-json', rs_path])

    content_mode = request_body.get('contentMode', 'normal')

    # LOD2 supplies the real 3D buildings; skip it entirely when buildings are excluded.
    if content_mode != 'no-buildings':
        lod2_path = _fetch_lod2_roofs(eff_area, work_dir)
        if lod2_path:
            args.extend(['--lod2-json', lod2_path])

    wt_path = _extract_water_towers(osm_path, eff_area, work_dir)
    if wt_path:
        args.extend(['--water-towers-json', wt_path])

    if request_body.get('withTerrain'):
        try:
            smoothing_amount = float(request_body.get('terrainSmoothing', 2))
        except (TypeError, ValueError):
            smoothing_amount = 2.0
        smoothing_amount = max(0.0, min(smoothing_amount, 10.0))
        elev_path = fetch_elevation(eff_area, work_dir, smoothing_amount=smoothing_amount)
        args.extend(['--elevation-json', elev_path])
        args.extend(['--lon-min', str(eff_area['lonMin']),
                     '--lon-max', str(eff_area['lonMax']),
                     '--lat-min', str(eff_area['latMin']),
                     '--lat-max', str(eff_area['latMax'])])

    if content_mode == 'no-buildings':
        args.append('--exclude-buildings')

    # User-adjustable feature-height factors (1.0 = built-in defaults).
    for key, flag in (('roadHeightFactor', '--road-height-factor'),
                      ('buildingHeightFactor', '--building-height-factor'),
                      ('baseHeightFactor', '--base-height-factor'),
                      ('waterDepthFactor', '--water-depth-factor'),
                      ('terrainHeightFactor', '--terrain-height-factor')):
        value = request_body.get(key)
        if value is not None:
            try:
                args.extend([flag, str(float(value))])
            except (TypeError, ValueError):
                pass

    if request_body.get('export3mf'):
        args.append('--export-3mf')

    if (not request_body.get('hideLocationMarker') and
            not request_body.get('multipartMode') and
            'marker1' in request_body):
        m = request_body['marker1']
        lon_range = eff_area['lonMax'] - eff_area['lonMin']
        lat_range = eff_area['latMax'] - eff_area['latMin']
        if lon_range > 0 and lat_range > 0:
            mx = (m['lon'] - eff_area['lonMin']) / lon_range
            my = (m['lat'] - eff_area['latMin']) / lat_range
            if 0.04 < mx < 0.96 and 0.04 < my < 0.96:
                args.extend(['--marker1', json.dumps({'x': mx, 'y': my})])

    args.append(osm_path)

    print('running: ' + ' '.join(args), flush=True)
    subprocess.check_call(args)


def copy_outputs(work_dir, out_dir, id_slug):
    os.makedirs(out_dir, exist_ok=True)
    file_map = {
        'map.stl': id_slug + '.stl',
        'map-ways.stl': id_slug + '-ways.stl',
        'map-rest.stl': id_slug + '-rest.stl',
        'map.svg': id_slug + '.svg',
        'map.blend': id_slug + '.blend',
        'map-content.json': id_slug + '.map-content.json',
    }
    # 3MF is optional (only when requested); copy it if produced.
    optional = {'map.3mf': id_slug + '.3mf'}
    for src_name, dst_name in file_map.items():
        src_path = os.path.join(work_dir, src_name)
        dst_path = os.path.join(out_dir, dst_name)
        if src_path != dst_path and os.path.exists(src_path):
            shutil.copy2(src_path, dst_path)
        elif not os.path.exists(src_path):
            print('warning: expected output not found: ' + src_name, flush=True)
    for src_name, dst_name in optional.items():
        src_path = os.path.join(work_dir, src_name)
        dst_path = os.path.join(out_dir, dst_name)
        if src_path != dst_path and os.path.exists(src_path):
            shutil.copy2(src_path, dst_path)


def main():
    parser = argparse.ArgumentParser(description='Run Touch Mapper pipeline locally (no AWS).')
    parser.add_argument('--request', required=True, help='Request JSON string')
    parser.add_argument('--data-root', required=True, help='Root of local-data/maps directory')
    args_parsed = parser.parse_args()

    request_body = json.loads(args_parsed.request)
    request_id = request_body['requestId']

    parts = request_id.split('/', 1)
    id_start = parts[0]
    id_slug = parts[1] if len(parts) > 1 else 'map'

    data_root = args_parsed.data_root
    info_path = os.path.join(data_root, 'info', id_start + '.json')
    work_dir = os.path.join(data_root, 'data', id_start)

    os.makedirs(work_dir, exist_ok=True)

    try:
        print('Starting local conversion for requestId: ' + request_id, flush=True)

        # Stage 1: fetch OSM data from Overpass / OSM API
        write_status(info_path, request_id, 20)
        fetch_osm(request_body['effectiveArea'], os.path.join(work_dir, 'map.osm'))

        # Stage 2: OSM -> 3D tactile model (OSM2World + clip-2d + Blender)
        write_status(info_path, request_id, 60)
        run_osm_to_tactile(work_dir, request_body)

        # Stage 3: generate map description JSON
        sys.path.insert(0, script_dir)
        import map_desc  # noqa: PLC0415
        meta_raw_path = os.path.join(work_dir, 'map-meta-raw.json')
        map_desc.run_map_desc(meta_raw_path, profile={})

        # Stage 4: rename outputs to <id_slug>.* so the server can serve them
        copy_outputs(work_dir, work_dir, id_slug)

        # Write final info.json with full metadata + status=100
        meta_path = os.path.join(work_dir, 'map-meta.json')
        meta = {}
        if os.path.exists(meta_path):
            with open(meta_path, 'r', encoding='utf-8') as f:
                meta = json.load(f)
        info = copy.copy(request_body)
        for key, value in meta.items():
            info[key] = value
        info['status'] = {'progress': 100}
        with open(info_path, 'w', encoding='utf-8') as f:
            json.dump(info, f, ensure_ascii=False)

        print('Conversion complete!', flush=True)

    except Exception as e:
        import traceback
        traceback.print_exc()
        write_status(info_path, request_id, 0, 'unknown', str(e))
        sys.exit(1)


if __name__ == '__main__':
    main()
