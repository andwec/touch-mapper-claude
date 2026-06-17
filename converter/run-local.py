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


def run_osm_to_tactile(work_dir, request_body):
    eff_area = request_body['effectiveArea']
    osm_path = os.path.join(work_dir, 'map.osm')

    args = [
        sys.executable, os.path.join(script_dir, 'osm-to-tactile.py'),
        '--scale', str(request_body['scale']),
        '--diameter', str(request_body['diameter']),
        '--size', str(request_body['size']),
    ]

    if request_body.get('noBorders'):
        args.append('--no-borders')

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
