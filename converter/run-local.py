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


def _smooth_elevation_grid(elevations, grid_size, passes=2):
    """Apply passes of 3×3 average smoothing to reduce DSM artifacts."""
    grid = [row[:] for row in elevations]
    for _ in range(passes):
        smoothed = [row[:] for row in grid]
        for r in range(1, grid_size - 1):
            for c in range(1, grid_size - 1):
                smoothed[r][c] = (
                    grid[r-1][c-1] + grid[r-1][c] + grid[r-1][c+1] +
                    grid[r  ][c-1] + grid[r  ][c] + grid[r  ][c+1] +
                    grid[r+1][c-1] + grid[r+1][c] + grid[r+1][c+1]
                ) / 9.0
        grid = smoothed
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


def fetch_elevation(eff_area, work_dir, grid_size=80):
    """Fetch elevation grid; primary = Terrain-RGB tiles (AWS, no key), fallback = APIs."""
    elevations = None

    # Primary: AWS Terrain-RGB (Terrarium) — free, no rate limit, no API key
    try:
        print('fetching elevation from terrain-rgb tiles (AWS) ...', flush=True)
        elevations = _fetch_terrain_rgb_elevations(eff_area, grid_size=grid_size)
        print('terrain-rgb: {}x{} grid ready'.format(grid_size, grid_size), flush=True)
    except Exception as e:
        print('terrain-rgb failed: {} — trying API fallbacks'.format(e), flush=True)

    # Fallback: OpenTopoData EUDEM → SRTM30m → Open-Elevation
    if elevations is None:
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

    elevations = _smooth_elevation_grid(elevations, grid_size)

    elev_path = os.path.join(work_dir, 'elevation.json')
    with open(elev_path, 'w', encoding='utf-8') as f:
        json.dump({'grid_size': grid_size, 'elevations': elevations}, f)
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

    if request_body.get('withTerrain'):
        elev_path = fetch_elevation(eff_area, work_dir)
        args.extend(['--elevation-json', elev_path])
        args.extend(['--lon-min', str(eff_area['lonMin']),
                     '--lon-max', str(eff_area['lonMax']),
                     '--lat-min', str(eff_area['latMin']),
                     '--lat-max', str(eff_area['latMax'])])

    content_mode = request_body.get('contentMode', 'normal')
    if content_mode == 'no-buildings':
        args.append('--exclude-buildings')

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
    for src_name, dst_name in file_map.items():
        src_path = os.path.join(work_dir, src_name)
        dst_path = os.path.join(out_dir, dst_name)
        if src_path != dst_path and os.path.exists(src_path):
            shutil.copy2(src_path, dst_path)
        elif not os.path.exists(src_path):
            print('warning: expected output not found: ' + src_name, flush=True)


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
