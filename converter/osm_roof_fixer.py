#!/usr/bin/env python3
"""
OSM Roof Fixer
Fügt roof:shape zu allen Gebäuden ohne Dach-Tag hinzu.
Unterstützt: ways, relations (multipolygon), building:part
Logik: Gebäudetyp → Geometrie (Ratio + Fläche) → Fallback

Verwendung:
  python3 osm_roof_fixer.py input.osm output.osm
"""

import xml.etree.ElementTree as ET
import math
import sys
from collections import defaultdict

# ─── Gebäudetyp-Regeln ───────────────────────────────────────────
ALWAYS_FLAT = {
    'industrial', 'warehouse', 'supermarket', 'retail', 'commercial',
    'garage', 'garages', 'carport', 'train_station', 'transportation',
    'parking', 'hangar', 'storage_tank'
}
ALWAYS_GABLED = {
    'house', 'detached', 'semidetached_house', 'terrace', 'bungalow',
    'cabin', 'farm', 'farm_auxiliary', 'barn', 'stable', 'shed', 'greenhouse'
}
PYRAMIDAL = {
    'church', 'chapel', 'cathedral', 'temple', 'mosque', 'synagogue'
}

# ─── Hilfsfunktionen ─────────────────────────────────────────────
def get_tags(elem):
    return {t.get('k'): t.get('v') for t in elem.findall('tag')}

def set_tag(elem, key, value):
    for t in elem.findall('tag'):
        if t.get('k') == key:
            t.set('v', value)
            return
    new_tag = ET.SubElement(elem, 'tag')
    new_tag.set('k', key)
    new_tag.set('v', value)

def get_coords(node_map, nd_refs):
    return [node_map[nd.get('ref')]
            for nd in nd_refs if nd.get('ref') in node_map]

def bbox_ratio_and_area(coords):
    if len(coords) < 3:
        return 1.0, 0.0
    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    dlat = (max(lats) - min(lats)) * 111320
    dlon = (max(lons) - min(lons)) * 111320 * math.cos(math.radians(sum(lats)/len(lats)))
    if dlon < 0.01:
        return 1.0, 0.0
    ratio = max(dlat, dlon) / max(min(dlat, dlon), 0.01)
    return ratio, dlat * dlon

def choose_roof(tags, coords):
    """Gibt (roof:shape, Begründung) zurück oder (None, reason) wenn bereits gesetzt."""
    if 'roof:shape' in tags:
        return None, "bereits gesetzt"

    btype = tags.get('building', tags.get('building:part', 'yes'))

    if btype in ALWAYS_FLAT:
        return 'flat', f"Typ={btype}"
    if btype in PYRAMIDAL:
        return 'pyramidal', f"Typ={btype}"
    if btype in ALWAYS_GABLED:
        return 'gabled', f"Typ={btype}"

    # Geometrie auswerten
    if coords and len(coords) >= 3:
        ratio, area = bbox_ratio_and_area(coords)
        if area > 1500:
            return 'flat', f"Fläche={area:.0f}m²>1500"
        if ratio > 2.5:
            return 'gabled', f"Ratio={ratio:.1f}>2.5"
        if ratio < 1.5:
            return 'hipped', f"Ratio={ratio:.1f}<1.5"
        return 'gabled', f"Ratio={ratio:.1f}"

    return 'gabled', "Fallback"

# ─── Hauptprogramm ───────────────────────────────────────────────
def main(input_path, output_path):
    print(f"Lese {input_path} ...")
    tree = ET.parse(input_path)
    root = tree.getroot()

    # Node-Koordinaten aufbauen
    node_map = {}
    for node in root.findall('node'):
        node_map[node.get('id')] = (float(node.get('lat')), float(node.get('lon')))

    stats = defaultdict(int)
    changes = []

    # ── Ways ──
    for way in root.findall('way'):
        tags = get_tags(way)
        is_building = 'building' in tags or 'building:part' in tags
        if not is_building:
            continue
        if 'roof:shape' in tags:
            stats['bereits_gesetzt'] += 1
            continue
        coords = get_coords(node_map, way.findall('nd'))
        shape, reason = choose_roof(tags, coords)
        if shape:
            set_tag(way, 'roof:shape', shape)
            stats[shape] += 1
            changes.append(f"  Way {way.get('id'):>12} [{tags.get('building', tags.get('building:part','?')):20}] → {shape} ({reason})")

    # ── Relations ──
    # Way-Index für schnellen Lookup bauen
    way_index = {w.get('id'): w for w in root.findall('way')}

    for rel in root.findall('relation'):
        tags = get_tags(rel)
        is_building = 'building' in tags or 'building:part' in tags
        if not is_building:
            continue
        if 'roof:shape' in tags:
            stats['bereits_gesetzt'] += 1
            continue
        coords = []
        for member in rel.findall('member'):
            if member.get('type') == 'way' and member.get('role') == 'outer':
                way = way_index.get(member.get('ref'))
                if way is not None:
                    coords.extend(get_coords(node_map, way.findall('nd')))
        shape, reason = choose_roof(tags, coords)
        if shape:
            set_tag(rel, 'roof:shape', shape)
            stats[shape] += 1
            changes.append(f"  Relation {rel.get('id'):>8} [{tags.get('building','?'):20}] → {shape} ({reason})")

    # ── Report ──
    if changes:
        print(f"\nNeu zugewiesene Dächer ({len(changes)}):")
        for c in changes:
            print(c)
    else:
        print("Keine Änderungen notwendig — alle Gebäude haben bereits roof:shape.")

    print(f"\nStatistik:")
    print(f"  Bereits gesetzt:  {stats['bereits_gesetzt']}")
    for shape in ['gabled', 'hipped', 'flat', 'pyramidal']:
        if stats[shape]:
            print(f"  Neu {shape:10}: {stats[shape]}")

    # ── Verifikation ──
    still_missing = []
    for elem in list(root.findall('way')) + list(root.findall('relation')):
        t = get_tags(elem)
        if ('building' in t or 'building:part' in t) and 'roof:shape' not in t:
            still_missing.append(f"{elem.tag}/{elem.get('id')} building={t.get('building','?')}")
    if still_missing:
        print(f"\n⚠ Noch {len(still_missing)} Gebäude ohne roof:shape:")
        for s in still_missing:
            print(f"  {s}")
    else:
        print(f"\n✓ Alle Gebäude haben roof:shape.")

    tree.write(output_path, encoding='unicode', xml_declaration=True)
    print(f"✓ Gespeichert: {output_path}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Verwendung: python3 osm_roof_fixer.py input.osm output.osm")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
