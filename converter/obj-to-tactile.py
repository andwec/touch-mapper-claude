# pyright: reportMissingImports=false
from __future__ import print_function
import sys
import argparse
import os
import bpy
import mathutils
import bmesh
import json
import math
import time

script_dir = os.path.dirname(__file__)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
import tactile_constants as tc

perf_clock = getattr(time, 'perf_counter', time.time)


class TerrainSampler:
    """Bilinear-interpolation sampler over a lat/lon elevation grid mapped to Blender XY coordinates."""

    def __init__(self, elevations, min_x, min_y, max_x, max_y, height_factor=1.0):
        self.grid = elevations  # [row][col], row = Y (lat) bottom→top, col = X (lon) left→right
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.grid_h = len(elevations)
        self.grid_w = len(elevations[0]) if elevations else 0
        all_elev = [e for row in elevations for e in row]
        self.min_elev = min(all_elev) if all_elev else 0.0
        # Vertical exaggeration: 1.0 = real heights, 0.0 = completely flat, >1 = exaggerated.
        self.height_factor = height_factor

    def sample(self, x, y):
        """Return terrain elevation in Blender units (meters) at world (x, y), min elevation = 0."""
        fx = (x - self.min_x) / max(self.max_x - self.min_x, 1e-9) * (self.grid_w - 1)
        fy = (y - self.min_y) / max(self.max_y - self.min_y, 1e-9) * (self.grid_h - 1)
        fx = max(0.0, min(self.grid_w - 1.0, fx))
        fy = max(0.0, min(self.grid_h - 1.0, fy))
        col0 = int(fx); col1 = min(col0 + 1, self.grid_w - 1)
        row0 = int(fy); row1 = min(row0 + 1, self.grid_h - 1)
        tx = fx - col0; ty = fy - row0
        e00 = self.grid[row0][col0] - self.min_elev
        e10 = self.grid[row0][col1] - self.min_elev
        e01 = self.grid[row1][col0] - self.min_elev
        e11 = self.grid[row1][col1] - self.min_elev
        elev = e00*(1-tx)*(1-ty) + e10*tx*(1-ty) + e01*(1-tx)*ty + e11*tx*ty
        return elev * self.height_factor


def create_terrain_solid(sampler, min_x, min_y, max_x, max_y, base_below_z, overlap=0.0, grid_size=40):
    """Create a closed solid terrain mesh (top surface + walls + flat bottom)."""
    top_verts = []
    bot_verts = []
    for row in range(grid_size):
        for col in range(grid_size):
            x = min_x + (max_x - min_x) * col / (grid_size - 1)
            y = min_y + (max_y - min_y) * row / (grid_size - 1)
            # Raise terrain surface by overlap so roads/buildings embed slightly into it
            z = sampler.sample(x, y) + overlap
            top_verts.append((x, y, z))
            bot_verts.append((x, y, base_below_z))

    n = grid_size * grid_size
    verts = top_verts + bot_verts
    faces = []

    def top(r, c): return r * grid_size + c
    def bot(r, c): return n + r * grid_size + c

    for row in range(grid_size - 1):
        for col in range(grid_size - 1):
            # Top surface (normal +Z → CCW from above)
            faces.append((top(row,col), top(row,col+1), top(row+1,col+1), top(row+1,col)))
            # Bottom surface (normal -Z → CW from above)
            faces.append((bot(row,col), bot(row+1,col), bot(row+1,col+1), bot(row,col+1)))

    last = grid_size - 1
    for col in range(grid_size - 1):  # front wall (row=0, normal -Y)
        faces.append((top(0,col), bot(0,col), bot(0,col+1), top(0,col+1)))
    for col in range(grid_size - 1):  # back wall (row=last, normal +Y)
        faces.append((top(last,col+1), bot(last,col+1), bot(last,col), top(last,col)))
    for row in range(grid_size - 1):  # left wall (col=0, normal -X)
        faces.append((top(row+1,0), bot(row+1,0), bot(row,0), top(row,0)))
    for row in range(grid_size - 1):  # right wall (col=last, normal +X)
        faces.append((top(row,last), bot(row,last), bot(row+1,last), top(row+1,last)))

    mesh = bpy.data.meshes.new('TerrainMesh')
    obj = bpy.data.objects.new('TerrainSolid', mesh)
    bpy.context.scene.collection.objects.link(obj)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')
    print("terrain solid created: {} verts {} faces".format(len(verts), len(faces)))
    return obj


def _densify_for_terrain(ob, max_edge):
    """Subdivide near-horizontal edges longer than max_edge so the object follows the terrain.

    Without this, a long road segment or building footprint edge is a straight chord
    between two far-apart vertices; the fine terrain surface bulges up between them and
    buries the feature. Vertical edges (e.g. building walls) have ~zero horizontal length
    and are left untouched, so geometry growth stays bounded.
    """
    max_edge_sq = max_edge * max_edge
    try:
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        for _ in range(10):
            bm.edges.ensure_lookup_table()
            long_edges = [
                e for e in bm.edges
                if ((e.verts[0].co.x - e.verts[1].co.x) ** 2 +
                    (e.verts[0].co.y - e.verts[1].co.y) ** 2) > max_edge_sq
            ]
            if not long_edges:
                break
            bmesh.ops.subdivide_edges(bm, edges=long_edges, cuts=1, use_grid_fill=False)
        bm.to_mesh(ob.data)
        bm.free()
        ob.data.update()
    except Exception as e:
        print("WARNING: terrain densify failed for {}: {}".format(ob.name, e))


def apply_terrain_to_objects(sampler, skip_buildings=False):
    """Raise all map mesh vertices by terrain elevation at their (x, y) position.

    Features are first densified so each road keeps a uniform height above the terrain
    surface instead of getting bridged over (buried) where the ground rises between
    sparse vertices. When skip_buildings is set, the 'Buildings' object is left alone
    (LOD2 buildings are seated on the terrain during construction instead of draped).
    """
    skip = ['TerrainSolid', 'Base', 'Borders', 'CornerInside', 'CornerTop']
    if skip_buildings:
        skip.append('Buildings')
    # Water areas are flat by definition — densifying them causes polygon explosion in
    # Blender 5.x due to changed subdivide_edges behaviour on large N-gons.
    skip_densify = skip + ['WaterAreas']
    # Densify to roughly the elevation-grid cell size — finer than that the terrain is
    # just linear interpolation, so there is nothing left to bury features. Keep a 2 m
    # floor so a very fine (≈1 m) terrain grid doesn't over-tessellate roads/buildings;
    # the terrain mesh itself still carries the full 1 m detail.
    cell_x = (sampler.max_x - sampler.min_x) / max(sampler.grid_w - 1, 1)
    cell_y = (sampler.max_y - sampler.min_y) / max(sampler.grid_h - 1, 1)
    max_edge = max(min(cell_x, cell_y), 2.0)
    densify = sampler.height_factor != 0.0  # nothing to bury when terrain is flat
    for ob in list(bpy.context.scene.objects):
        if ob.type != 'MESH' or ob.name in skip:
            continue
        if densify and ob.name not in skip_densify:
            _densify_for_terrain(ob, max_edge)
        # Objects have identity transform in this pipeline, so world coords == local coords
        mesh = ob.data
        for v in mesh.vertices:
            v.co.z += sampler.sample(v.co.x, v.co.y)
        mesh.update()
    bpy.context.view_layer.update()
    print("terrain displacement applied (densify max_edge={:.2f}m, factor={})".format(
        max_edge, sampler.height_factor))


class BuildingHeightLookup:
    """Spatial lookup: given Blender (x, y) returns building height in Blender units (meters).
    Heights are clamped: minimum = default (4mm print), maximum = 20mm print height.
    1 Blender unit = 1 metre = 1000/scale mm in print.
    """

    def __init__(self, bh_data, min_x, min_y, max_x, max_y, default_height_units, scale):
        self.default = default_height_units
        self.max_height = 20.0 * scale / 1000  # 20mm print → Blender units
        lon_min = bh_data['lon_min']; lon_max = bh_data['lon_max']
        lat_min = bh_data['lat_min']; lat_max = bh_data['lat_max']
        self.entries = []
        dx = max_x - min_x; dy = max_y - min_y
        dlon = lon_max - lon_min; dlat = lat_max - lat_min
        for b in bh_data.get('buildings', []):
            if dlon > 0 and dlat > 0:
                bx = min_x + (b['lon'] - lon_min) / dlon * dx
                by = min_y + (b['lat'] - lat_min) / dlat * dy
                self.entries.append((bx, by, b['height_m']))

    def lookup(self, cx, cy, max_dist=30.0):
        """Return height in Blender units for building centroid (cx, cy)."""
        best_dist = max_dist * max_dist
        best_h = None
        for bx, by, h in self.entries:
            d2 = (bx - cx) ** 2 + (by - cy) ** 2
            if d2 < best_dist:
                best_dist = d2
                best_h = h
        if best_h is None:
            return self.default
        # Clamp: at least default height, at most 20mm print height
        return max(self.default, min(self.max_height, best_h))


class WaterTowerPlacer:
    """Maps water tower lat/lon from JSON to Blender (x, y) coordinates."""

    def __init__(self, wt_data, min_x, min_y, max_x, max_y):
        lon_min = wt_data['lon_min']; lon_max = wt_data['lon_max']
        lat_min = wt_data['lat_min']; lat_max = wt_data['lat_max']
        dx = max_x - min_x; dy = max_y - min_y
        dlon = lon_max - lon_min; dlat = lat_max - lat_min
        self.positions = []
        for t in wt_data.get('towers', []):
            if dlon > 0 and dlat > 0:
                bx = min_x + (t['lon'] - lon_min) / dlon * dx
                by = min_y + (t['lat'] - lat_min) / dlat * dy
                self.positions.append((bx, by))


class RoofShapeLookup:
    """Spatial lookup: given Blender (x, y) returns roof shape string for nearest building."""

    def __init__(self, rs_data, min_x, min_y, max_x, max_y):
        lon_min = rs_data['lon_min']; lon_max = rs_data['lon_max']
        lat_min = rs_data['lat_min']; lat_max = rs_data['lat_max']
        dx = max_x - min_x; dy = max_y - min_y
        dlon = lon_max - lon_min; dlat = lat_max - lat_min
        self.entries = []
        for b in rs_data.get('buildings', []):
            if dlon > 0 and dlat > 0:
                bx = min_x + (b['lon'] - lon_min) / dlon * dx
                by = min_y + (b['lat'] - lat_min) / dlat * dy
                self.entries.append((bx, by, b['roof_shape']))

    def lookup(self, cx, cy, max_dist=30.0):
        best_dist = max_dist * max_dist
        best_shape = 'pyramidal'
        for bx, by, shape in self.entries:
            d2 = (bx - cx) ** 2 + (by - cy) ** 2
            if d2 < best_dist:
                best_dist = d2
                best_shape = shape
        return best_shape


def _mesh_from_lod2_faces(name, faces_xyz, weld_eps=0.02):
    """Build one closed Blender mesh from a list of polygon faces.

    Vertices closer than weld_eps (metres) are merged so the separate LOD2
    boundary surfaces (ground, walls, roof) stitch into a single watertight
    solid. Returns the new object, or None if no usable face was created.
    """
    bm = bmesh.new()
    vmap = {}

    def vert_at(x, y, z):
        key = (round(x / weld_eps), round(y / weld_eps), round(z / weld_eps))
        v = vmap.get(key)
        if v is None:
            v = bm.verts.new((x, y, z))
            vmap[key] = v
        return v

    for face in faces_xyz:
        verts = []
        for (x, y, z) in face:
            v = vert_at(x, y, z)
            # Skip consecutive duplicate vertices (common in GML rings).
            if not verts or verts[-1] is not v:
                verts.append(v)
        # GML rings repeat the first point as the last; drop the closing repeat.
        if len(verts) >= 2 and verts[0] is verts[-1]:
            verts.pop()
        if len(verts) < 3:
            continue
        try:
            bm.faces.new(verts)
        except ValueError:
            pass  # duplicate face — ignore

    if not bm.faces:
        bm.free()
        return None

    # Weld any remaining coincident vertices, then make normals point outward.
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=weld_eps)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))

    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    ob = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def _place_building_on_terrain(ob, sampler, base_extra_m=0.5):
    """Seat a finished building solid on the terrain without burying or floating it.

    The footprint (the building's lowest ring) is dropped to just below the LOWEST
    terrain under the building, while everything above is lifted to clear the HIGHEST
    terrain under it. Result: walls/roof always stand fully above ground (never
    buried), the downhill base reaches down into the terrain (no gap to print), and
    the building stays a clean vertical solid (no shearing/tilting).
    """
    mesh = ob.data
    if not mesh.vertices:
        return
    min_z = min(v.co.z for v in mesh.vertices)
    eps = 0.02
    foot = [sampler.sample(v.co.x, v.co.y) for v in mesh.vertices if v.co.z - min_z < eps]
    if not foot:
        return
    t_min = min(foot)
    t_max = max(foot)
    floor_z = t_min - base_extra_m
    for v in mesh.vertices:
        if v.co.z - min_z < eps:
            v.co.z = floor_z       # footprint sits below all terrain → fuses with the slab
        else:
            v.co.z += t_max        # structure lifted clear of the highest ground
    mesh.update()


def build_lod2_buildings(lod2_data, min_x, min_y, max_x, max_y, scale, terrain_sampler=None):
    """Create solid building meshes directly from LOD2 geometry.

    Each LOD2 building is rebuilt as a complete closed solid (ground + walls +
    roof) at its true, undistorted map position, so the roof always lines up with
    the walls and the result is watertight for 3D printing. Any building whose
    footprint touches the map is built in full (even if it straddles the edge);
    the part outside the map is removed later by a clean rectangular clip, so edge
    buildings are never bent or squashed. Returns the list of created objects.
    """
    lon_min = lod2_data['lon_min']; lon_max = lod2_data['lon_max']
    lat_min = lod2_data['lat_min']; lat_max = lod2_data['lat_max']
    dlon = max(lon_max - lon_min, 1e-10)
    dlat = max(lat_max - lat_min, 1e-10)
    dx = max_x - min_x; dy = max_y - min_y
    mm_to_units = scale / 1000
    max_height_units = 20.0 * mm_to_units       # cap silhouette at 20 mm print
    sink = tc.BUILDING_BASE_SINK_MM * mm_to_units  # push base into the plate to fuse

    def ll_to_xy(lon, lat):
        x = min_x + (lon - lon_min) / dlon * dx
        y = min_y + (lat - lat_min) / dlat * dy
        return x, y

    created = []
    clip_box = None
    for b in lod2_data.get('buildings', []):
        base_elev = b['base_elev']
        ridge = 0.0
        bxmin = bymin = 1e18
        bxmax = bymax = -1e18
        raw_faces = []
        for flat in b.get('faces', []):
            verts = []
            i = 0
            while i + 2 < len(flat):
                vx, vy = ll_to_xy(flat[i], flat[i + 1])
                vz = flat[i + 2] - base_elev
                verts.append((vx, vy, vz))
                if vz > ridge:
                    ridge = vz
                if vx < bxmin: bxmin = vx
                if vx > bxmax: bxmax = vx
                if vy < bymin: bymin = vy
                if vy > bymax: bymax = vy
                i += 3
            if len(verts) >= 3:
                raw_faces.append(verts)
        if not raw_faces:
            continue
        # Keep any building whose footprint overlaps the map (so straddling buildings
        # are complete); drop only those fully outside. The overhang is clipped below.
        if bxmax < min_x or bxmin > max_x or bymax < min_y or bymin > max_y:
            continue

        zfactor = (max_height_units / ridge) if ridge > max_height_units and ridge > 0 else 1.0

        # True coordinates — no clamping, so the building keeps its real shape.
        faces_xyz = [[(vx, vy, vz * zfactor - sink) for (vx, vy, vz) in verts]
                     for verts in raw_faces]

        ob = _mesh_from_lod2_faces('LOD2Building', faces_xyz)
        if ob is None:
            continue

        # Only buildings that cross the map edge need cutting; cut each one on its own
        # (a single clean solid) — far more reliable than a boolean on the joined mesh.
        if bxmin < min_x or bxmax > max_x or bymin < min_y or bymax > max_y:
            # Bracket the clip box's Z range to this building's real height (with a small
            # margin). A non-watertight LOD2 mesh can make the EXACT boolean return the
            # whole box; bounding Z to the building keeps that failure from producing a
            # giant spike instead of a ~tall-as-the-building artefact.
            bzmin = min(vz for face in faces_xyz for (_, _, vz) in face)
            bzmax = max(vz for face in faces_xyz for (_, _, vz) in face)
            clip_box = create_cube(min_x, min_y, max_x, max_y, bzmin - 1.0, bzmax + 1.0)
            clip_box.name = 'BuildingClipBox'
            _boolean_intersect(ob, clip_box)
            bpy.data.objects.remove(clip_box, do_unlink=True)
            clip_box = None
            if not ob.data.polygons:           # fully cut away
                bpy.data.objects.remove(ob, do_unlink=True)
                continue
        # Seat the finished solid on the terrain (after any clip), so it never buries.
        if terrain_sampler is not None:
            _place_building_on_terrain(ob, terrain_sampler)
        created.append(ob)

    if clip_box is not None:
        bpy.data.objects.remove(clip_box, do_unlink=True)
    return created


def _boolean_intersect(ob, box):
    """Clip ob to the volume of box with an EXACT boolean intersect (applied in place)."""
    try:
        bpy.ops.object.select_all(action='DESELECT')
        bpy.context.view_layer.objects.active = ob
        ob.select_set(True)
        mod = ob.modifiers.new('Clip', 'BOOLEAN')
        mod.operation = 'INTERSECT'
        mod.object = box
        try:
            mod.solver = 'EXACT'
        except Exception:
            pass
        bpy.ops.object.modifier_apply(modifier=mod.name)
    except Exception as e:
        print("WARNING: building clip failed for {}: {}".format(ob.name, e))


def create_water_tower(x, y, shaft_height, shaft_radius, tank_radius):
    """Create a water tower at (x, y): cylinder shaft + sphere tank."""
    bpy.ops.mesh.primitive_cylinder_add(
        radius=shaft_radius,
        depth=shaft_height,
        location=(x, y, shaft_height / 2),
        vertices=12,
        end_fill_type='TRIFAN',
    )
    shaft = bpy.context.active_object
    shaft.name = 'WaterTowerShaft'

    tank_z = shaft_height + tank_radius
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=tank_radius,
        location=(x, y, tank_z),
        segments=12,
        ring_count=8,
    )
    tank = bpy.context.active_object
    tank.name = 'WaterTowerTank'

    bpy.ops.object.select_all(action='DESELECT')
    shaft.select_set(True)
    tank.select_set(True)
    bpy.context.view_layer.objects.active = shaft
    bpy.ops.object.join()
    result = bpy.context.active_object
    result.name = 'WaterTower'
    return result


def _split_mesh_by_components(source_obj):
    """Split a joined mesh into one object per connected component using bmesh (no bpy.ops)."""
    src_mesh = source_obj.data
    bm = bmesh.new()
    bm.from_mesh(src_mesh)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    # BFS to find connected components (sets of face indices)
    visited_verts = set()
    components = []
    for start in bm.verts:
        if start.index in visited_verts:
            continue
        comp_verts = set()
        comp_faces = set()
        queue = [start]
        while queue:
            v = queue.pop()
            if v.index in visited_verts:
                continue
            visited_verts.add(v.index)
            comp_verts.add(v.index)
            for edge in v.link_edges:
                for ov in edge.verts:
                    if ov.index not in visited_verts:
                        queue.append(ov)
            for face in v.link_faces:
                comp_faces.add(face.index)
        if comp_faces:
            components.append(comp_faces)
    bm.free()

    parts = []
    all_face_indices = set(range(len(src_mesh.polygons)))
    for comp_faces in components:
        bm2 = bmesh.new()
        bm2.from_mesh(src_mesh)
        bm2.faces.ensure_lookup_table()
        to_del = [f for f in bm2.faces if f.index not in comp_faces]
        bmesh.ops.delete(bm2, geom=to_del, context='FACES')
        new_mesh = bpy.data.meshes.new('BuildingPart')
        bm2.to_mesh(new_mesh)
        bm2.free()
        new_mesh.update()
        new_obj = bpy.data.objects.new('BuildingPart', new_mesh)
        new_obj.location = source_obj.location.copy()
        bpy.context.scene.collection.objects.link(new_obj)
        parts.append(new_obj)

    bpy.data.objects.remove(source_obj, do_unlink=True)
    return parts


def extrude_buildings_with_heights(joined_obj, height_lookup, default_height, roof_height=0.0,
                                   roof_shape_lookup=None):
    """Split joined building mesh by connected components, extrude each to its OSM height and roof shape."""
    parts = _split_mesh_by_components(joined_obj)
    print("separated {} building parts for individual extrusion".format(len(parts)))

    for ob in parts:
        bpy.context.view_layer.objects.active = ob
        bbox = ob.bound_box
        cx = (bbox[0][0] + bbox[6][0]) / 2 + ob.location.x
        cy = (bbox[0][1] + bbox[6][1]) / 2 + ob.location.y
        h = height_lookup.lookup(cx, cy) if height_lookup else default_height
        shape = roof_shape_lookup.lookup(cx, cy) if roof_shape_lookup else 'pyramidal'

        extrude_building(ob, h)
        if roof_height > 0:
            try:
                apply_roof(ob, shape, roof_height)
            except Exception as e:
                print("WARNING: roof failed ({}) at ({:.1f},{:.1f}): {}".format(shape, cx, cy, e))
        fatten(ob)

    return parts


# Try the Blender 2.78 bundled svgwrite location first; if it doesn't exist
# (e.g. newer Blender or custom install), fall back to system Python path.
_b278_svgwrite = os.path.join(script_dir, 'blender', '2.78', 'python', 'lib', 'python3.5', 'svgwrite')
if os.path.isdir(_b278_svgwrite):
    sys.path.insert(1, _b278_svgwrite)
# These modules imported at the site of use


def do_cmdline():
    parser = argparse.ArgumentParser(description='''Read OSM map meshes, modify to tactile map, and export as .stl''')
    parser.add_argument('--min-x', metavar='FLOAT', type=float, help='minimum X bound')
    parser.add_argument('--min-y', metavar='FLOAT', type=float, help='minimum Y bound')
    parser.add_argument('--max-x', metavar='FLOAT', type=float, help='maximum X bound')
    parser.add_argument('--max-y', metavar='FLOAT', type=float, help='maximum Y bound')
    parser.add_argument('--no-stl-export', action='store_true', help='do not export to .stl file')
    parser.add_argument('--scale', metavar='N', type=int, help="scale to export STL in, 4000 would mean one Blender unit (meter) = 0.25mm (STL file unit is normally mm)")
    parser.add_argument('--marker1', metavar='MARKER', help="first marker's position relative to top left corner")
    parser.add_argument('--diameter', metavar='METERS', type=int, help="larger of map area x and y diameter in meters")
    parser.add_argument('--size', metavar='METERS', type=float, help="print size in cm")
    parser.add_argument('--no-borders', action='store_true', help="don't draw borders around the edges")
    parser.add_argument('--export-wireframe-png', action='store_true', help="export orthographic top-view wireframe PNG")
    parser.add_argument('--base-path', help='base output path (without extension), defaults to first input path')
    parser.add_argument('--building-heights-json', help='path to JSON with per-building heights from OSM')
    parser.add_argument('--water-towers-json', help='path to JSON with water tower positions from OSM')
    parser.add_argument('--roof-shapes-json', help='path to JSON with per-building roof shapes from OSM')
    parser.add_argument('--lod2-json', help='path to JSON with LOD2 building geometries from NRW WFS')
    parser.add_argument('--elevation-json', help='path to elevation grid JSON for terrain')
    parser.add_argument('--lon-min', type=float, help='map longitude minimum (for terrain)')
    parser.add_argument('--lon-max', type=float, help='map longitude maximum')
    parser.add_argument('--lat-min', type=float, help='map latitude minimum')
    parser.add_argument('--lat-max', type=float, help='map latitude maximum')
    # User-adjustable feature-height factors (1.0 = built-in default heights).
    parser.add_argument('--road-height-factor', type=float, default=1.0, help='scale road/rail heights')
    parser.add_argument('--building-height-factor', type=float, default=1.0, help='scale default (non-LOD2) building heights')
    parser.add_argument('--base-height-factor', type=float, default=1.0, help='scale base plate thickness')
    parser.add_argument('--water-depth-factor', type=float, default=1.0, help='scale water/waterway depth')
    parser.add_argument('--terrain-height-factor', type=float, default=1.0, help='scale terrain elevation (0 = flat)')
    parser.add_argument('--export-3mf', action='store_true', help='also write a 3MF with each element as a separate object')
    parser.add_argument('mesh_paths', metavar='PATHS', nargs='+', help='.obj/.ply files to use as input')
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    return args

def print_verts(ob):
    for v in ob.data.vertices:
        print(ob.name, ob.matrix_world @ mathutils.Vector(v.co))

def get_minimum_coordinate(ob):
    min_x, min_y, min_z, _max_x, _max_y, _max_z = get_object_world_bounds(ob)
    return (min_x, min_y, min_z)


def get_object_world_bounds(ob):
    bbox_corners = [ob.matrix_world @ mathutils.Vector(corner) for corner in ob.bound_box]
    min_x = 1000000
    min_y = 1000000
    min_z = 1000000
    max_x = -1000000
    max_y = -1000000
    max_z = -1000000
    for corner in bbox_corners:
        min_x = min(min_x, corner[0])
        min_y = min(min_y, corner[1])
        min_z = min(min_z, corner[2])
        max_x = max(max_x, corner[0])
        max_y = max(max_y, corner[1])
        max_z = max(max_z, corner[2])
    return (min_x, min_y, min_z, max_x, max_y, max_z)

def move_everything(move_by):
    vector = mathutils.Vector(move_by)
    for ob in bpy.context.scene.objects:
        if ob.type == 'MESH':
            ob.location += vector

def all_mesh_objects():
    out = []
    for ob in bpy.context.scene.objects:
        if ob.type != 'MESH':
            continue
        if ob.name == 'map':
            # Happens when there is nothing on the map
            continue
        out.append(ob)
    return out

def rgb(r, g, b):
    return 'rgb(%d, %d, %d)' % (round(r*2.55), round(g*2.55), round(b*2.55))

def add_polygons(dwg, g, ob):
    mesh = ob.data
    verts = mesh.vertices
    for polygon in mesh.polygons:
        points = []
        for vert_index in polygon.vertices:
            world_co = ob.matrix_world @ verts[vert_index].co
            points.append(('%.1f' % world_co[0], '%.1f' % world_co[1]))
        g.add(dwg.polygon(points=points))

def add_svg_object(dwg, main_g, ob, color):
    g = dwg.g(stroke=color, fill=color)
    g['stroke-width'] = 0.3 # removes gaps between objects
    main_g.add(g)

    if ob.name.startswith('Road'):
        g['stroke-width'] = 0.8 # Make roads a bit thicker so embosser draws them
    add_polygons(dwg, g, ob)

def add_road_overlay_object(dwg, main_g, ob):
    g = dwg.g(opacity=0.0, fill='red', stroke='blue')
    g['stroke-width'] = 5.0
    main_g.add(g)

    add_polygons(dwg, g, ob)

def export_svg(base_path, args):
    t = perf_clock()
    min_x, min_y, max_x, max_y = (args.min_x, args.min_y, args.max_x, args.max_y)
    one_cm_units = (max_y - min_y) / args.size

    try:
        import svgwrite
    except ImportError:
        try:
            import subprocess
            print("svgwrite not found, installing...")
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--quiet', 'svgwrite==1.1.9'])
            import svgwrite
        except Exception as e:
            print("WARNING: svgwrite unavailable, skipping SVG export: " + str(e))
            return
    dwg = svgwrite.Drawing(base_path + '.svg', profile = 'basic')
    dwg['width']  = "%.2f" % (args.size) + 'cm'
    dwg['height'] = "%.2f" % (args.size + 1) + 'cm'
    dwg['viewBox'] = "%f %f %f %f" % (min_x, min_y - one_cm_units, max_x - min_x, max_y - min_y + one_cm_units)
    dwg['shape-rendering'] = 'geometricPrecision'
    dwg['stroke-linejoin'] = 'round' # greatly reduces protruding edges caused by non-zero stroke-width

    # Group objects into different layers
    objs = all_mesh_objects()
    buildings = []
    roads_car = []
    roads_ped = []
    rails = []
    rivers = []
    water_areas = []
    for ob in objs:
        try:
            if ob.name.startswith('Road'):
                if is_pedestrian(ob.name):
                    roads_ped.append(ob)
                else:
                    roads_car.append(ob)
            elif ob.name.startswith('Rail'):
                rails.append(ob)
            elif ob.name.startswith('Waterway') or ob.name.startswith('River'):
                rivers.append(ob)
            elif ob.name.startswith('Water') or ob.name.startswith('AreaFountain'):
                water_areas.append(ob)
            elif ob.name.startswith('Building'):
                buildings.append(ob)
            else:
                print("UNHANDLED TYPE IN SVG CREATION: " + ob.name)
        except Exception as e:
            print("SVG export failed {}: {}".format(ob.name, str(e)))

    # White background
    dwg.add(dwg.rect(insert=(min_x - 5, min_y - 5 - one_cm_units), size=(max_x - min_x + 10, max_y - min_y + 10 + one_cm_units), fill=rgb(100, 100, 100)))

    # A group for main content
    clip_path = dwg.defs.add(dwg.clipPath(id='main_clip'))
    clip_path.add(dwg.rect(insert=(min_x, min_y), size=(max_x - min_x, max_y - min_y)))
    main_g = dwg.add(dwg.g(clip_path='url(#main_clip)'))

    for ob in rails:
        add_svg_object(dwg, main_g, ob, rgb(0, 50, 0))
    for ob in rivers:
        add_svg_object(dwg, main_g, ob, rgb(20, 20, 100))
    for ob in water_areas:
        add_svg_object(dwg, main_g, ob, rgb(20, 20, 100))
    for ob in roads_car:
        add_svg_object(dwg, main_g, ob, rgb(70, 0, 0))
    for ob in roads_ped:
        add_svg_object(dwg, main_g, ob, rgb(0, 0, 0))
    for ob in buildings:
        add_svg_object(dwg, main_g, ob, rgb(80, 20, 100))

    # Add overlays
    for ob in objs:
        try:
            if ob.name.startswith('Road') or ob.name.startswith('Rail') or ob.name.startswith('Waterway') or ob.name.startswith('River'):
                add_road_overlay_object(dwg, main_g, ob)
        except Exception as e:
            print("SVG export failed2 {}: {}".format(ob.name, str(e)))

    # Add north marker to top-right corner
    g = dwg.g(fill='black')
    g['stroke-width'] = 0
    g.set_desc('North-east corner')
    g.add(dwg.polygon(points=[
        ('%.2f' % (max_x),                      '%.2f' % (min_y - one_cm_units*0.3)),
        ('%.2f' % (max_x - one_cm_units*0.7),   '%.2f' % (min_y - one_cm_units*0.3)),
        ('%.2f' % (max_x - one_cm_units*0.7/2), '%.2f' % (min_y - one_cm_units)),
    ]))
    dwg.add(g)

    dwg.save()
    print("creating SVG took " + (str(perf_clock() - t)))

def _export_stl(stl_path, scale):
    print("creating {stl}...".format(stl=stl_path))
    try:
        bpy.ops.export_mesh.stl(filepath=stl_path, check_existing=False,
                                axis_forward='Y', axis_up='Z', global_scale=(1000 / scale),
                                use_selection=True)
    except AttributeError:
        # Blender 4.x+: new operator — selection flag renamed to export_selected_objects
        bpy.ops.wm.stl_export(filepath=stl_path, check_existing=False,
                              forward_axis='Y', up_axis='Z', global_scale=(1000 / scale),
                              export_selected_objects=True)

def export_stl(base_path, scale):
    bpy.ops.object.select_all(action='SELECT')
    _export_stl(base_path + '.stl', scale)

def export_stl_separate(base_path, scale):
    bpy.ops.object.select_all(action='DESELECT')
    for ob in bpy.context.scene.objects:
        ob.select_set(ob.name.endswith('Roads') or ob.name.endswith('RoadAreas') or ob.name.endswith('Rails'))
    _export_stl(base_path + '-ways.stl', scale)
    bpy.ops.object.select_all(action='INVERT')
    _export_stl(base_path + '-rest.stl', scale)

def _3mf_group_for(name):
    """Map a scene object name to one of the user-facing 3MF element groups."""
    if name == 'SelectedAddress':
        return 'Marker'
    if name.startswith('Building') or name.startswith('WaterTower'):
        return 'Buildings'
    if name.startswith('Rail') or name.endswith('Roads') or name.endswith('RoadAreas'):
        return 'Roads'
    if 'Waterway' in name or name.startswith('River') or name in ('WaterAreas', 'JoinedWaterways'):
        return 'Water'
    if name.startswith('Base') or name.startswith('Border') or name.startswith('Corner'):
        return 'Terrain'
    return 'Other'


def _object_triangles(ob, factor):
    """Return (verts, tris) for one object, triangulated and scaled to print mm."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.verts.ensure_lookup_table()
    index = {}
    verts = []
    for i, v in enumerate(bm.verts):
        index[v] = i
        verts.append((v.co.x * factor, v.co.y * factor, v.co.z * factor))
    tris = [(index[f.verts[0]], index[f.verts[1]], index[f.verts[2]])
            for f in bm.faces if len(f.verts) == 3]
    bm.free()
    return verts, tris


def export_3mf(base_path, scale):
    """Write a single .3mf containing each map element as a separate named object.

    Groups (Roads / Terrain / Water / Buildings / Marker) become distinct 3MF
    objects so a slicer can assign each its own colour/filament or print settings.
    """
    import zipfile
    from xml.sax.saxutils import escape

    factor = 1000.0 / scale  # Blender metres -> print millimetres (matches STL export)

    # Preserve a friendly, stable object order in the file.
    order = ['Terrain', 'Roads', 'Water', 'Buildings', 'Marker', 'Other']
    grouped = {}
    for ob in bpy.context.scene.objects:
        if ob.type != 'MESH' or not ob.data.polygons:
            continue
        grouped.setdefault(_3mf_group_for(ob.name), []).append(ob)

    objects_xml = []
    build_xml = []
    oid = 0
    for group in order:
        members = grouped.get(group)
        if not members:
            continue
        all_verts = []
        all_tris = []
        base = 0
        for ob in members:
            verts, tris = _object_triangles(ob, factor)
            if not tris:
                continue
            all_verts.extend(verts)
            all_tris.extend((a + base, b + base, c + base) for (a, b, c) in tris)
            base += len(verts)
        if not all_tris:
            continue
        oid += 1
        v_xml = ''.join('<vertex x="{:.4f}" y="{:.4f}" z="{:.4f}"/>'.format(*v) for v in all_verts)
        t_xml = ''.join('<triangle v1="{}" v2="{}" v3="{}"/>'.format(*t) for t in all_tris)
        objects_xml.append(
            '<object id="{}" type="model" name="{}"><mesh><vertices>{}</vertices>'
            '<triangles>{}</triangles></mesh></object>'.format(oid, escape(group), v_xml, t_xml))
        build_xml.append('<item objectid="{}"/>'.format(oid))

    if not objects_xml:
        print("3MF: no geometry to export")
        return

    model = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        '<resources>' + ''.join(objects_xml) + '</resources>'
        '<build>' + ''.join(build_xml) + '</build></model>')
    content_types = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
        '</Types>')
    rels = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
        '</Relationships>')

    path = base_path + '.3mf'
    print("creating {} ...".format(path))
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', content_types)
        z.writestr('_rels/.rels', rels)
        z.writestr('3D/3dmodel.model', model)
    print("3MF written with {} separate objects".format(len(objects_xml)))


def _make_object_manifold(ob):
    """Weld, drop interior faces, fix normals and close holes so the mesh is manifold.

    LOD2 buildings in particular pick up a few interior/shared wall faces that leave
    edges shared by >2 faces; OrcaSlicer / Bambu Studio flag those as non-manifold.
    """
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.001)
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.mesh.select_mode(type='FACE')
    bpy.ops.mesh.select_interior_faces()
    bpy.ops.mesh.delete(type='FACE')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.fill_holes(sides=0)
    bpy.ops.object.mode_set(mode='OBJECT')


def print_mesh_stats(label):
    objs = [(ob.name, len(ob.data.polygons), len(ob.data.vertices))
            for ob in bpy.data.objects if ob.type == 'MESH']
    objs.sort(key=lambda x: -x[1])
    total_polys = sum(p for _, p, _ in objs)
    print("\n=== mesh stats: {} ===".format(label))
    for name, polys, verts in objs:
        print("  {:30s}  {:>10,} polys  {:>10,} verts".format(name, polys, verts))
    print("  {:30s}  {:>10,} polys total".format("TOTAL", total_polys))
    print("")


def cleanup_meshes_for_export():
    """Make every map mesh manifold before export (skips the terrain slab, which is
    a clean grid by construction, to save time on its large face count)."""
    skip = ('Base', 'TerrainSolid')
    try:
        bpy.ops.object.mode_set(mode='OBJECT')
    except Exception:
        pass
    for ob in list(bpy.context.scene.objects):
        if ob.type != 'MESH' or ob.name in skip or not ob.data.polygons:
            continue
        try:
            _make_object_manifold(ob)
        except Exception as e:
            try:
                bpy.ops.object.mode_set(mode='OBJECT')
            except Exception:
                pass
            print("WARNING: manifold cleanup failed for {}: {}".format(ob.name, e))


def export_blend_file(base_path):
    blend_path = base_path + '.blend'
    bpy.ops.object.select_all(action='SELECT') # it's handy to have everything selected initially
    bpy.ops.wm.save_as_mainfile(filepath=blend_path, check_existing=False, compress=True)


def export_wireframe_png(base_path, output_name, min_x, min_y, max_x, max_y):
    try:
        _export_wireframe_png_impl(base_path, output_name, min_x, min_y, max_x, max_y)
    except Exception as e:
        print("WARNING: wireframe PNG not supported in this Blender version: " + str(e))

def _export_wireframe_png_impl(base_path, output_name, min_x, min_y, max_x, max_y):
    t = time.time()
    wireframe_path = base_path + '-' + output_name + '.png'
    width = max_x - min_x
    height = max_y - min_y
    ortho_scale = max(width, height) * tc.WIREFRAME_CAMERA_PADDING
    if ortho_scale < tc.WIREFRAME_CAMERA_MIN_SCALE:
        ortho_scale = tc.WIREFRAME_CAMERA_MIN_SCALE
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    source_meshes = all_mesh_objects()
    max_z = 0.0
    for source in source_meshes:
        (_obj_min_x, _obj_min_y, _obj_min_z, _obj_max_x, _obj_max_y, obj_max_z) = get_object_world_bounds(source)
        max_z = max(max_z, obj_max_z)
    camera_height = max_z + max(ortho_scale, 10.0)

    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_RENDER'
    scene.render.alpha_mode = 'TRANSPARENT'
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'
    scene.render.resolution_x = tc.WIREFRAME_RENDER_RESOLUTION
    scene.render.resolution_y = tc.WIREFRAME_RENDER_RESOLUTION
    scene.render.resolution_percentage = 100
    scene.render.filepath = wireframe_path
    if scene.world is not None:
        scene.world.horizon_color = (0.0, 0.0, 0.0)

    # Best effort: mirror viewport intent on Blender versions that expose overlay controls.
    for screen in bpy.data.screens:
        for area in screen.areas:
            for space in area.spaces:
                overlay = getattr(space, 'overlay', None)
                if overlay is not None and hasattr(overlay, 'show_wireframes'):
                    overlay.show_wireframes = True
                shading = getattr(space, 'shading', None)
                if shading is not None and hasattr(shading, 'type'):
                    shading.type = 'SOLID'

    bpy.ops.object.camera_add()
    camera = bpy.context.active_object
    camera.location = (center_x, center_y, camera_height)
    camera.rotation_euler = (0.0, 0.0, 0.0)
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = ortho_scale
    camera.data.clip_start = 0.01
    camera.data.clip_end = max(1000.0, camera_height + ortho_scale * 4)
    scene.camera = camera

    solid_material = bpy.data.materials.new('SolidRenderMaterial')
    solid_material.type = 'SURFACE'
    solid_material.use_shadeless = True
    solid_material.diffuse_color = (0.72, 0.72, 0.72)
    solid_material.specular_intensity = 0.0

    wire_material = bpy.data.materials.new('WireframeRenderMaterial')
    wire_material.type = 'WIRE'
    wire_material.use_shadeless = True
    wire_material.diffuse_color = (0.0, 0.0, 0.0)
    wire_overlay_objects = []
    solid_overlay_objects = []
    hidden_originals = []
    for ob in source_meshes:
        hidden_originals.append((ob, ob.hide_render))
        ob.hide_render = True

        solid_ob = ob.copy()
        solid_ob.data = ob.data.copy()
        scene.collection.objects.link(solid_ob)
        solid_ob.hide_render = False
        mesh = solid_ob.data
        if len(mesh.materials) == 0:
            mesh.materials.append(solid_material)
        else:
            for material_index in range(len(mesh.materials)):
                mesh.materials[material_index] = solid_material
        solid_overlay_objects.append(solid_ob)

        wire_ob = solid_ob.copy()
        wire_ob.data = solid_ob.data.copy()
        scene.collection.objects.link(wire_ob)
        wire_ob.hide_render = False
        wire_mesh = wire_ob.data
        if len(wire_mesh.materials) == 0:
            wire_mesh.materials.append(wire_material)
        else:
            for material_index in range(len(wire_mesh.materials)):
                wire_mesh.materials[material_index] = wire_material
        wire_ob.location[2] += 0.0001
        wire_overlay_objects.append(wire_ob)

    bpy.ops.render.render(write_still=True)

    bpy.ops.object.select_all(action='DESELECT')
    for ob in solid_overlay_objects:
        ob.select_set(True)
    for ob in wire_overlay_objects:
        ob.select_set(True)
    camera.select_set(True)
    bpy.context.view_layer.objects.active = camera
    bpy.ops.object.delete()
    for (ob, old_hide_render) in hidden_originals:
        ob.hide_render = old_hide_render
    print("creating wireframe PNG took %.2f" % (time.time() - t))

def create_cube(min_x, min_y, max_x, max_y, min_z, max_z):
    x1 = min(min_x, max_x)
    x2 = max(min_x, max_x)
    y1 = min(min_y, max_y)
    y2 = max(min_y, max_y)
    z1 = min(min_z, max_z)
    z2 = max(min_z, max_z)

    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent()
    bpy.ops.object.mode_set(mode = 'OBJECT')
    cube.location = [ (x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2 ]
    cube.scale = [ (x2 - x1) / 2, (y2 - y1) / 2, (z2 - z1) / 2 ]
    bpy.context.view_layer.update() # flush changes to location and scale
    return cube

def add_borders(min_x, min_y, max_x, max_y, width, bottom, height, corner_height):
    borders = []
    borders.append(create_cube(min_x, min_y, min_x + width, max_y, bottom, height))
    borders.append(create_cube(max_x, min_y, max_x - width, max_y, bottom, height))
    borders.append(create_cube(min_x + width*0.99, min_y, max_x - width*0.99, min_y + width, bottom, height))
    borders.append(create_cube(min_x + width*0.99, max_y, max_x - width*0.99, max_y - width, bottom, height))
    join_objects(borders, 'Borders')

    # Marker for north-east corner
    create_cube(max_x - width*0.99, max_y - width*0.99, max_x - width*2.7, max_y - width*2.7, 0, height).name = 'CornerInside'
    create_cube(max_x, max_y, max_x - width*3, max_y - width*3, height*0.99, corner_height).name = 'CornerTop'

def create_bounds(min_x, min_y, max_x, max_y, scale, no_borders, base_height_factor=1.0):
    mm_to_units = scale / 1000
    if not no_borders:
        add_borders(min_x, min_y, max_x, max_y, tc.BORDER_WIDTH_MM * mm_to_units, \
                    0, tc.BORDER_HEIGHT_MM * mm_to_units, (tc.BUILDING_HEIGHT_MM + 1) * mm_to_units)
    base_height = tc.BASE_HEIGHT_MM * mm_to_units * base_height_factor
    overlap = tc.BASE_OVERLAP_MM * mm_to_units # move cube this much up so that it overlaps enough with objects they merge into one object
    base_cube = create_cube(min_x, min_y, max_x, max_y, -base_height + overlap, overlap)
    base_cube.name = 'Base'
    return base_cube

def add_marker1(args, scale):
    min_x, min_y, max_x, max_y = (args.min_x, args.min_y, args.max_x, args.max_y)
    if args.marker1 == 'center':
        marker_x, marker_y = (0.5, 0.5)
    else:
        coords = json.loads(args.marker1)
        marker_x = float(coords['x'])
        marker_y = float(coords['y'])
        
    mm_to_units = scale / 1000
    radius = tc.MARKER_RADIUS_MM * mm_to_units
    height = tc.MARKER_HEIGHT_MM * mm_to_units
    # If the cone has sharp top, three.js won't render it remotely properly, and it'll 3D print poorly too
    bpy.ops.mesh.primitive_cone_add(vertices = 16, radius1 = radius, radius2 = radius / 8, depth = height, \
        location = [ min_x + (max_x - min_x) * marker_x, min_y + (max_y - min_y) * marker_y, height / 2 ])
    bpy.context.active_object.name = 'SelectedAddress'

def remove_everything():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def mesh_name_for_path(mesh_path):
    basename = os.path.basename(mesh_path).lower()
    if 'road-areas-ped' in basename:
        return 'RoadAreaGroup::pedestrian'
    if 'road-areas-car' in basename:
        return 'RoadAreaGroup'
    if 'roads-ped' in basename:
        return 'RoadGroup::pedestrian'
    if 'roads-car' in basename:
        return 'RoadGroup'
    if 'rails' in basename:
        return 'RailGroup'
    if 'buildings' in basename:
        return 'BuildingGroup'
    if 'waterways' in basename:
        return 'WaterwayGroup'
    if 'water-areas' in basename:
        return 'WaterGroup'
    if 'other' in basename:
        return 'OtherGroup'
    return 'MeshGroup'


def imported_meshes_since(old_names):
    out = []
    for ob in bpy.context.scene.objects:
        if ob.type != 'MESH':
            continue
        if ob.name in old_names:
            continue
        out.append(ob)
    return out


def import_mesh_file(mesh_path):
    t = perf_clock()
    old_names = set((ob.name for ob in bpy.context.scene.objects if ob.type == 'MESH'))
    extension = os.path.splitext(mesh_path)[1].lower()
    if extension == '.obj':
        try:
            # Blender 3.3+ new operator
            bpy.ops.wm.obj_import(filepath=mesh_path, forward_axis='NEGATIVE_Z', up_axis='Y')
        except AttributeError:
            bpy.ops.import_scene.obj(filepath=mesh_path, axis_forward='-Z', axis_up='Y')
    elif extension == '.ply':
        try:
            # Blender 4.x new operator
            bpy.ops.wm.ply_import(filepath=mesh_path)
        except AttributeError:
            if not hasattr(bpy.ops.import_mesh, 'ply'):
                bpy.ops.wm.addon_enable(module='io_mesh_ply')
            bpy.ops.import_mesh.ply(filepath=mesh_path)
    else:
        raise Exception("unsupported mesh extension: " + extension)

    imported = imported_meshes_since(old_names)
    target_name = mesh_name_for_path(mesh_path)
    for i, ob in enumerate(imported):
        if i == 0:
            ob.name = target_name
        else:
            ob.name = target_name + ('_%03d' % i)

# Extrude floor to a flat-roofed building
def extrude_building(ob, height):
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.extrude_region_move(TRANSFORM_OT_translate={ "value": (0.0, 0.0, height) })
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent()
    bpy.ops.object.mode_set(mode = 'OBJECT')

def _boundary_verts_ordered(boundary_edges):
    """Traverse boundary edges to get footprint vertices in polygon order."""
    if not boundary_edges:
        return []
    adj = {}
    for e in boundary_edges:
        v0, v1 = e.verts[0], e.verts[1]
        adj.setdefault(v0, []).append(v1)
        adj.setdefault(v1, []).append(v0)
    for neighbors in adj.values():
        if len(neighbors) != 2:
            return []  # non-manifold boundary
    start = boundary_edges[0].verts[0]
    ordered = [start]
    prev, current = None, start
    while True:
        nxt = next((n for n in adj[current] if n is not prev), None)
        if nxt is None or nxt is start:
            break
        ordered.append(nxt)
        prev, current = current, nxt
    return ordered


def _polygon_area_2d(verts):
    """Shoelace formula for 2D projected area of a polygon (uses x/y, ignores z)."""
    n = len(verts)
    if n < 3:
        return 0.0
    area = sum(verts[i].co.x * verts[(i+1) % n].co.y -
               verts[(i+1) % n].co.x * verts[i].co.y for i in range(n))
    return abs(area) * 0.5


def add_pyramid_roof(ob, roof_height):
    """Replace all top faces with a single-apex pyramid (no gap between triangulated faces)."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    if not bm.verts:
        bm.free()
        return

    max_z = max(v.co.z for v in bm.verts)
    eps = 0.001
    top_faces = [f for f in bm.faces if all(abs(v.co.z - max_z) < eps for v in f.verts)]
    if not top_faces:
        bm.free()
        return

    top_idx_set = set(f.index for f in top_faces)
    all_top_edges = {e for f in top_faces for e in f.edges}

    # Boundary edges border exactly one top face (= outer perimeter of roof region).
    # Interior edges border two top faces (= triangulation diagonals, must be removed).
    boundary_edges = [e for e in all_top_edges
                      if sum(1 for af in e.link_faces if af.index in top_idx_set) == 1]
    interior_edges = [e for e in all_top_edges
                      if sum(1 for af in e.link_faces if af.index in top_idx_set) == 2]

    # Single apex at centroid of all top vertices
    top_verts = {v for f in top_faces for v in f.verts}
    cx = sum(v.co.x for v in top_verts) / len(top_verts)
    cy = sum(v.co.y for v in top_verts) / len(top_verts)

    bmesh.ops.delete(bm, geom=top_faces, context='FACES_ONLY')
    # Interior edges are now orphaned (no faces); skip deletion — they don't appear in STL
    # and deleting already-dereferenced faces via 'EDGES' context can crash Blender.

    apex = bm.verts.new((cx, cy, max_z + roof_height))
    for edge in boundary_edges:
        v0, v1 = edge.verts[0], edge.verts[1]
        try:
            bm.faces.new([v0, v1, apex])
        except Exception:
            pass

    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.update()

def add_gabled_roof(ob, roof_height):
    """Add a gabled roof: ridge along the principal (long) axis found via PCA."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    if not bm.verts:
        bm.free()
        return

    max_z = max(v.co.z for v in bm.verts)
    eps = 0.001
    top_faces = [f for f in bm.faces if all(abs(v.co.z - max_z) < eps for v in f.verts)]
    if not top_faces:
        bm.free()
        return

    top_idx_set = set(f.index for f in top_faces)
    all_top_edges = {e for f in top_faces for e in f.edges}

    boundary_edges = [e for e in all_top_edges
                      if sum(1 for af in e.link_faces if af.index in top_idx_set) == 1]
    interior_edges = [e for e in all_top_edges
                      if sum(1 for af in e.link_faces if af.index in top_idx_set) == 2]

    top_verts = list({v for f in top_faces for v in f.verts})
    xs = [v.co.x for v in top_verts]
    ys = [v.co.y for v in top_verts]
    n = len(xs)
    cx = sum(xs) / n
    cy = sum(ys) / n

    # PCA: find the principal (long) axis of the building footprint in 2-D
    cxx = sum((x - cx) ** 2 for x in xs) / n
    cyy = sum((y - cy) ** 2 for y in ys) / n
    cxy = sum((x - cx) * (y - cy) for x, y in zip(xs, ys)) / n
    tr = cxx + cyy
    disc = max(0.0, (tr / 2) ** 2 - (cxx * cyy - cxy * cxy))
    lam = tr / 2 + math.sqrt(disc)        # largest eigenvalue
    if abs(cxy) > 1e-10:
        dx, dy = lam - cyy, cxy
    elif cxx >= cyy:
        dx, dy = 1.0, 0.0
    else:
        dx, dy = 0.0, 1.0
    length = math.sqrt(dx * dx + dy * dy)
    if length <= 1e-10:
        # Degenerate (nearly square or tiny building): fall back to pyramid
        bm.free()
        add_pyramid_roof(ob, roof_height)
        return
    dx /= length
    dy /= length

    # Project all top vertices onto the long axis to find ridge endpoints
    projs = [(v.co.x - cx) * dx + (v.co.y - cy) * dy for v in top_verts]
    p_min, p_max = min(projs), max(projs)
    p_mid = (p_min + p_max) / 2

    # Rectangularity check: compare actual footprint area to PCA bounding box.
    # Non-rectangular buildings (L/T/U shapes) get a pyramid instead.
    ordered_verts = _boundary_verts_ordered(boundary_edges)
    if ordered_verts:
        poly_area = _polygon_area_2d(ordered_verts)
        q_projs = [(v.co.x - cx) * (-dy) + (v.co.y - cy) * dx for v in ordered_verts]
        pca_bbox_area = (p_max - p_min) * max(max(q_projs) - min(q_projs), 1e-6)
        if poly_area / max(pca_bbox_area, 1e-6) < 0.65:
            bm.free()
            add_pyramid_roof(ob, roof_height)
            return

    r0 = bm.verts.new((cx + dx * p_min, cy + dy * p_min, max_z + roof_height))
    r1 = bm.verts.new((cx + dx * p_max, cy + dy * p_max, max_z + roof_height))

    bmesh.ops.delete(bm, geom=top_faces, context='FACES_ONLY')
    # Interior edges are now orphaned; skip deletion — harmless for STL, avoids crash.

    def ridge_vert(v):
        proj = (v.co.x - cx) * dx + (v.co.y - cy) * dy
        return r0 if proj < p_mid else r1

    for edge in boundary_edges:
        v0, v1 = edge.verts[0], edge.verts[1]
        rv0 = ridge_vert(v0)
        rv1 = ridge_vert(v1)
        try:
            if rv0 is rv1:
                # Gable triangle: both verts on same side of ridge
                bm.faces.new([v0, v1, rv0])
            else:
                # Slope face: split into two triangles to avoid non-planar quad
                bm.faces.new([v0, v1, rv1])
                bm.faces.new([v0, rv1, rv0])
        except Exception:
            pass

    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.update()


def apply_roof(ob, shape, roof_height):
    """Dispatch to the right roof generator based on OSM roof:shape."""
    if shape == 'flat':
        return
    if shape in ('gabled', 'hipped'):
        add_gabled_roof(ob, roof_height)
    else:
        add_pyramid_roof(ob, roof_height)


def join_selected(name):
    combined = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = combined
    combined.name = name
    bpy.ops.object.join()
    return combined

def join_objects(objects, name):
    if len(objects) == 0:
        return None
    if len(objects) == 1:
        bpy.ops.object.select_all(action='DESELECT')
        objects[0].select_set(True)
        bpy.context.view_layer.objects.active = objects[0]
        objects[0].name = name
        return objects[0]
    bpy.ops.object.select_all(action='DESELECT')
    for ob in objects:
        ob.select_set(True)
    return join_selected(name)

def raise_ob(objs, height):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = objs
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.extrude_region_move(TRANSFORM_OT_translate={ "value": (0.0, 0.0, height) })
    bpy.ops.object.mode_set(mode = 'OBJECT')

def water_remesh_and_extrude(object, extrude_height):
    # Extrude just enough that remeshing works
    bpy.context.view_layer.objects.active = object
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.extrude_region_move(TRANSFORM_OT_translate={ "value": (0.0, 0.0, extrude_height) })
    bpy.ops.mesh.normals_make_consistent()
    bpy.ops.object.mode_set(mode = 'OBJECT')

    # Remesh
    max_dimension = max(object.dimensions[0], object.dimensions[1])
    depth = min(max(math.log2(max_dimension) - 1, 2), 8) # Max vertex distance == 2m => max dimension 128 == remesh depth 6 (or so)
    modifier = object.modifiers.new('Modifier', 'REMESH')
    # Blender 4.x+ defaults the REMESH modifier to VOXEL mode, where octree_depth is
    # ignored and a fixed voxel_size (0.1 m) is used instead — that remeshes a large
    # water area at 0.1 m resolution and explodes the poly count into the millions.
    # Force a mode that actually honours octree_depth.
    modifier.mode = 'SHARP'
    modifier.octree_depth = math.ceil(depth)
    modifier.use_remove_disconnected = False
    bpy.ops.object.modifier_apply(modifier=modifier.name)

def water_wave_pattern(object, depth, scale):
    extrude_height = 1.0
    water_remesh_and_extrude(object, extrude_height)

    # Start creating wave pattern
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bm = bmesh.from_edit_mesh(object.data)
    bm.verts.ensure_lookup_table()

    # Record x,y positions of edge verts (verts of non-horizontal edges)
    edge_verts = {}
    for edge in bm.edges:
        verts = edge.verts
        if abs(verts[0].co.z - verts[1].co.z) > extrude_height / 2:
            edge_verts[str(verts[0].co.x) + ',' + str(verts[0].co.y)] = True

    # Set top verts' z positions. Bottom verts are at 0.
    density = math.pi * 2 / tc.WATER_WAVE_DISTANCE_MM / (scale/1000) 
    for v in bm.verts:
        if v.co.z > extrude_height / 2:
            min_height = -10000
            if str(v.co.x) + ',' + str(v.co.y) in edge_verts:
                min_height = depth / 4
            v.co.z = max(min_height, (math.sin(v.co.x * density) + math.sin(v.co.y * density)) * depth / 4 + depth / 2)
        else:
            v.co.z = 0
    bmesh.update_edit_mesh(object.data, loop_triangles=False, destructive=False)

    bpy.ops.object.mode_set(mode = 'OBJECT')

def is_pedestrian(road_name):
    return road_name.endswith('::pedestrian')

## Disable stdout buffering
#class Unbuffered(object):
#   def __init__(self, stream):
#       self.stream = stream
#   def write(self, data):
#       self.stream.write(data)
#       self.stream.flush()
#   def __getattr__(self, attr):
#       return getattr(self.stream, attr)
#
#sys.stdout = Unbuffered(sys.stdout)


# Join edges that seem to form two ends of the same logical road or railway
def join_matching_edges(ob, min_x, min_y, max_x, max_y):
    lt = 0.2  # length difference + -
    dt = 0.15  # max distance 
    at = 0.5  # max sin(angle)  (30°)
    
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    from math import sin
    bm = bmesh.from_edit_mesh( bpy.context.object.data )
    bm.edges.ensure_lookup_table()
    
    center   = lambda e : ( e.verts[0].co + e.verts[1].co ) / 2
    length   = lambda e : ( e.verts[0].co - e.verts[1].co ).length
    dist     = lambda v1, v2: (  v2 -  v1 ).length
    sinAngle = lambda e1, e2: abs(sin((e1.verts[1].co - e1.verts[0].co).angle(e2.verts[1].co - e2.verts[0].co)))

    def point_between_edge_neighbor_verts(e):
        # Return middle of the verts adjacent to the edge
        verts = []
        for v in e.verts:
            for linked_e in v.link_edges:
                verts.extend((vv for vv in linked_e.verts if vv != e.verts[0] and vv != e.verts[1]))
        if len(verts) != 2:
            #print("edge has non-2 adjacent verts: " + str(len(verts)))
            return None
        return ((verts[0].co[0] + verts[1].co[0]) / 2, \
                (verts[0].co[1] + verts[1].co[1]) / 2, \
                (verts[0].co[2] + verts[1].co[2]) / 2)
    
    class CEdge:
        def __init__(self, e, into_edge):
            self.e = e
            self.center = center(e)
            self.length = length(e)
            self.into_edge = into_edge
            self.welded = False
    
    # Lengthen an edge that is supposedly at the end of a road, in an attempt to make roads'
    # widths consistent, instead of being the more narrow the greater the angle of their end edge.
    radians_90degrees = math.pi / 2
    def lengthen_edges(ce1, ce2):
        for ce in (ce1, ce2):
            verts = ce.e.verts
            edge_v = mathutils.Vector(verts[0].co) - mathutils.Vector(verts[1].co)
            angle = ce.into_edge.angle(edge_v)
            if abs(angle - radians_90degrees) > radians_90degrees / 9:
                multiplier = 1 / math.sin(angle)
                if multiplier > 3:
                    continue
                else:
                    verts[0].co = ce.center + (verts[0].co - ce.center) * multiplier
                    verts[1].co = ce.center + (verts[1].co - ce.center) * multiplier
    
    def filter_edges(edges):
        out = []
        for e in edges:
            if len(e.link_faces) != 1:
                continue
            # Because roads are clipped at the edges, funny coincidences can happen, so ignore those edges
            c = center(e)
            if abs(c[0] - min_x) < 0.1 or abs(c[0] - max_x) < 0.1 or abs(c[2] - min_y) < 0.1 or abs(c[2] - max_y) < 0.1:
                continue
            point_between_edges = point_between_edge_neighbor_verts(e)
            if not point_between_edges:
                continue
            vector_into_edge_face = center(e) - mathutils.Vector(point_between_edges)
            if vector_into_edge_face.length == 0:
                continue
            out.append(CEdge(e, vector_into_edge_face / vector_into_edge_face.length))
        return out
    candidate_edges = filter_edges(bm.edges)

    # Index edges into search tree
    edge_index_to_ce = {} # enable finding CEdge by edge
    kd = mathutils.kdtree.KDTree(len(candidate_edges))
    for i, ce in enumerate(candidate_edges):
        kd.insert(ce.center, i)
        edge_index_to_ce[ce.e.index] = ce
    kd.balance()

    def mark_all_t_junction_edges_welded(cedge):
        face_edges = cedge.e.link_faces[0].edges
        if len(face_edges) == 6:
            # Faces with 6 edges are probably T junctions. If we allow multiple roads
            # to connect to them, we often get a road that intersects itself (because X junctions are disabled in OSM2World)
            for fe in face_edges:
                fe_ce = edge_index_to_ce.get(fe.index, None)
                if fe_ce:
                    fe_ce.welded = True

    to_weld = {}
    for i, ce in enumerate(candidate_edges[:-1]):
        if ce.welded:
            continue
        ce.welded = True
        lmin = ce.length - lt
        lmax = ce.length + lt
        matches = []
        for (_co, oe_index, _dist) in kd.find_range(ce.center, dt):
            oe = candidate_edges[oe_index]
            if not oe.welded and lmin < oe.length < lmax and sinAngle(ce.e, oe.e) < at:
                turn_angle = ce.into_edge.angle(-oe.into_edge)
                if turn_angle > math.pi * 0.6: # pi * 0.5 is 90%
                    #print("not merging edges (%s, %s) pointing to opposite directions, angle is %f" % (ce.e, oe.e, turn_angle))
                    continue
                matches.append(oe)
                oe.welded = True
        
        if len(matches) == 1:
            # Join nothing where >2 ways meet, else all roads in the scene may become joined and intersect itself
            ev1, ev2 = ce.e.verts[:]
            oev1, oev2 = matches[0].e.verts[:]
            if dist(ev1.co, oev1.co) < dist(ev1.co, oev2.co) :
                if ev1 != oev1: to_weld[ev1] = oev1
                if ev2 != oev2: to_weld[ev2] = oev2
            else :
                if ev1 != oev2: to_weld[ev1] = oev2
                if ev2 != oev1: to_weld[ev2] = oev1
                # TODO: move welded verts to locations between the originals?
            lengthen_edges(ce, matches[0])
            mark_all_t_junction_edges_welded(ce)
            mark_all_t_junction_edges_welded(matches[0])
            
    print("%s: melding %d out of %d edges" % (ob.name, len(to_weld) / 2, len(bm.edges)))
    bmesh.ops.weld_verts(bm, targetmap = to_weld)
    bmesh.update_edit_mesh(bpy.context.object.data, loop_triangles=True)
    bpy.ops.object.mode_set(mode = 'OBJECT')

# Decimating gets rid of useless and harmful lane edges, as well as changing
# tris to n-gons (important to find edge's "direction")
def decimate(ob):
    # Decimating gets rid of useless lanes
    bpy.context.view_layer.objects.active = ob
    modifier = ob.modifiers.new('Modifier', 'DECIMATE')
    modifier.decimate_type = 'DISSOLVE'
    bpy.ops.object.modifier_apply(modifier=modifier.name)

# Fatten slightly to cause overlap and avoid faces too close to each other
def fatten(ob):
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.mode_set(mode = 'EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.transform.shrink_fatten(value=-0.05) # less than this and programs start to "remove double vertices"
    bpy.ops.object.mode_set(mode = 'OBJECT')

def do_ways(ways, height, min_x, min_y, max_x, max_y):
    if ways == None:
        return
    t = perf_clock()
    decimate(ways)
    join_matching_edges(ways, min_x, min_y, max_x, max_y)
    raise_ob(ways, height)
    fatten(ways)
    print("processing %s took %.2f" % (ways.name, perf_clock() - t))

def do_road_areas(roads, height):
    if roads == None:
        return
    t = perf_clock()
    decimate(roads)
    raise_ob(roads, height)
    fatten(roads)
    #print("processing %s took %.2f" % (roads.name, perf_clock() - t))

def process_objects(min_x, min_y, max_x, max_y, scale, no_borders, building_height_lookup=None, roof_shape_lookup=None, lod2_data=None,
                    road_height_factor=1.0, building_height_factor=1.0, water_depth_factor=1.0, terrain_sampler=None):
    t = perf_clock()
    mm_to_units = scale / 1000
    if not no_borders:
        space = (tc.BORDER_WIDTH_MM - tc.BORDER_HORIZONTAL_OVERLAP_MM) * mm_to_units 
        min_x = min_x + space
        min_y = min_y + space
        max_x = max_x - space
        max_y = max_y - space

    # First find out everything that we can join together into combined objects and do join,
    # because CPU usage is dominated by each Blender operation iterating through every object in the scene.
    roads_car = []
    roads_ped = []
    road_areas_car = []
    road_areas_ped = []
    buildings = []
    rails = []
    waterways = []
    water_areas = []
    deleteables = []
    for ob in all_mesh_objects():
        if ob.name.startswith('BuildingEntrance'):
            deleteables.append(ob)
        elif ob.name.startswith('Building'):
            buildings.append(ob)
        elif ob.name.startswith('Road'):
            if is_pedestrian(ob.name):
                if ob.name.startswith('RoadArea'):
                    road_areas_ped.append(ob)
                else:
                    roads_ped.append(ob)
            else:
                if ob.name.startswith('RoadArea'):
                    road_areas_car.append(ob)
                else:
                    roads_car.append(ob)
        elif ob.name.startswith('Rail'):
            rails.append(ob)
        elif ob.name.startswith('Waterway') or ob.name.startswith('River'):
            waterways.append(ob)
        elif ob.name.startswith('Water') or ob.name.startswith('AreaFountain'):
            water_areas.append(ob)
        else:
            print("UNHANDLED OBJECT TYPE: " + ob.name)
    print("initial steps took %.2f" % (perf_clock() - t))

    # Delete
    t = perf_clock()
    if len(deleteables) > 0:
        bpy.ops.object.select_all(action='DESELECT')
        for ob in deleteables:
            ob.select_set(True)
        bpy.ops.object.delete()
        #print("deleting %d objects took %.2f" % (len(deleteables), perf_clock() - t))

    # Pre-join stuff for performance
    joined_roads_car = join_objects(roads_car, 'CarRoads')
    joined_roads_ped = join_objects(roads_ped, 'PedestrianRoads')
    joined_road_areas_car = join_objects(road_areas_car, 'CarRoadAreas')
    joined_road_areas_ped = join_objects(road_areas_ped, 'PedestrianRoadAreas')
    joined_rails = join_objects(rails, 'Rails')
    joined_buildings = join_objects(buildings, 'Buildings')

    # Buildings — building_height_factor scales the default (non-LOD2) box + roof heights.
    roof_height = tc.BUILDING_ROOF_HEIGHT_MM * mm_to_units * building_height_factor
    default_building_height = tc.BUILDING_HEIGHT_MM * mm_to_units * building_height_factor
    if lod2_data is not None:
        # Real 3D buildings: rebuild each as a solid directly from LOD2 geometry,
        # discarding the flat OSM footprints (the LOD2 model already has walls +
        # roof aligned, and is watertight for printing).
        t = perf_clock()
        if joined_buildings is not None:
            bpy.data.objects.remove(joined_buildings, do_unlink=True)
        lod2_objs = build_lod2_buildings(lod2_data, min_x, min_y, max_x, max_y, scale,
                                         terrain_sampler=terrain_sampler)
        if lod2_objs:
            # Edge buildings are already cut flush per-building inside build_lod2_buildings.
            join_objects(lod2_objs, 'Buildings')
        print("built {} LOD2 solid buildings in {:.2f} s".format(len(lod2_objs), perf_clock() - t))
    elif joined_buildings:
        t = perf_clock()
        if building_height_lookup is not None or roof_shape_lookup is not None:
            # Per-component: individual height and/or roof shape per building
            parts = extrude_buildings_with_heights(
                joined_buildings, building_height_lookup,
                default_height=default_building_height,
                roof_height=roof_height,
                roof_shape_lookup=roof_shape_lookup)
            if parts:
                join_objects(parts, 'Buildings')
            print("processed {} buildings in {:.2f} s".format(len(parts), perf_clock() - t))
        else:
            extrude_building(joined_buildings, default_building_height)
            add_pyramid_roof(joined_buildings, roof_height)
            fatten(joined_buildings)
            print("processing {} buildings took {:.2f}".format(len(buildings), perf_clock() - t))

    # Waters — water_depth_factor scales how deep water/waterways are carved.
    t = perf_clock()
    if len(waterways) > 0:
        joined_waterways = join_objects(waterways, 'JoinedWaterways')
        raise_ob(joined_waterways, tc.WATERWAY_DEPTH_MM * mm_to_units * water_depth_factor)
    if len(water_areas):
        for water in water_areas:
            water_wave_pattern(water, tc.WATER_AREA_DEPTH_MM * mm_to_units * water_depth_factor, scale)
        join_objects(water_areas, 'WaterAreas')
    print("processing waters took %.2f" % (perf_clock() - t))

    # Rails — road_height_factor scales road/rail heights.
    if joined_rails != None:
        do_ways(joined_rails, tc.ROAD_HEIGHT_CAR_MM * mm_to_units * 0.99 * road_height_factor, min_x, min_y, max_x, max_y) # 0.99 to avoid faces in the same coordinates with roads

    # Roads — all road types share one height (no car/pedestrian distinction).
    road_height = tc.ROAD_HEIGHT_CAR_MM * mm_to_units * road_height_factor
    do_road_areas(joined_road_areas_car, road_height)
    do_road_areas(joined_road_areas_ped, road_height)
    do_ways(joined_roads_car, road_height, min_x, min_y, max_x, max_y)
    do_ways(joined_roads_ped, road_height, min_x, min_y, max_x, max_y)

def make_tactile_map(args):
    t = perf_clock()
    min_x, min_y, max_x, max_y = (args.min_x, args.min_y, args.max_x, args.max_y)

    bh_json = getattr(args, 'building_heights_json', None)
    elev_json = getattr(args, 'elevation_json', None)

    building_height_lookup = None
    if bh_json and os.path.exists(bh_json):
        try:
            with open(bh_json, 'r', encoding='utf-8') as f:
                bh_data = json.load(f)
            building_height_lookup = BuildingHeightLookup(
                bh_data, min_x, min_y, max_x, max_y,
                default_height_units=tc.BUILDING_HEIGHT_MM * (args.scale / 1000),
                scale=args.scale)
            print("building heights loaded: {} entries".format(len(bh_data.get('buildings', []))))
        except Exception as e:
            print("WARNING: building heights load failed: " + str(e))

    rs_json = getattr(args, 'roof_shapes_json', None)
    roof_shape_lookup = None
    if rs_json and os.path.exists(rs_json):
        try:
            with open(rs_json, 'r', encoding='utf-8') as f:
                rs_data = json.load(f)
            roof_shape_lookup = RoofShapeLookup(rs_data, min_x, min_y, max_x, max_y)
            print("roof shapes loaded: {} entries".format(len(rs_data.get('buildings', []))))
        except Exception as e:
            print("WARNING: roof shapes load failed: " + str(e))

    lod2_json = getattr(args, 'lod2_json', None)
    lod2_data = None
    if lod2_json and os.path.exists(lod2_json):
        try:
            with open(lod2_json, 'r', encoding='utf-8') as f:
                lod2_data = json.load(f)
            print("LOD2: {} buildings loaded".format(len(lod2_data.get('buildings', []))))
        except Exception as e:
            print("WARNING: LOD2 load failed: " + str(e))
            lod2_data = None

    # Load terrain early so LOD2 buildings can be seated on it during construction
    # (placed as clean vertical solids that never sink under the ground).
    terrain_sampler = None
    if elev_json and os.path.exists(elev_json):
        try:
            with open(elev_json, 'r', encoding='utf-8') as f:
                elev_data = json.load(f)
            terrain_sampler = TerrainSampler(
                elev_data['elevations'], min_x, min_y, max_x, max_y,
                height_factor=getattr(args, 'terrain_height_factor', 1.0))
            print("terrain sampler ready, min_elev={:.1f}m, grid={}x{}".format(
                terrain_sampler.min_elev, terrain_sampler.grid_h, terrain_sampler.grid_w))
        except Exception as e:
            print("WARNING: terrain load failed: " + str(e))
            terrain_sampler = None

    process_objects(min_x, min_y, max_x, max_y, args.scale, args.no_borders,
                    building_height_lookup=building_height_lookup,
                    roof_shape_lookup=roof_shape_lookup,
                    lod2_data=lod2_data,
                    road_height_factor=getattr(args, 'road_height_factor', 1.0),
                    building_height_factor=getattr(args, 'building_height_factor', 1.0),
                    water_depth_factor=getattr(args, 'water_depth_factor', 1.0),
                    terrain_sampler=terrain_sampler)
    print("process_objects() took " + (str(perf_clock() - t)))

    # Water towers
    wt_json = getattr(args, 'water_towers_json', None)
    if wt_json and os.path.exists(wt_json):
        try:
            with open(wt_json, 'r', encoding='utf-8') as f:
                wt_data = json.load(f)
            placer = WaterTowerPlacer(wt_data, min_x, min_y, max_x, max_y)
            mm_to_units = args.scale / 1000
            shaft_height = tc.BUILDING_HEIGHT_MM * mm_to_units
            shaft_radius = tc.WATER_TOWER_SHAFT_RADIUS_MM * mm_to_units
            tank_radius = tc.WATER_TOWER_TANK_RADIUS_MM * mm_to_units
            wt_objects = []
            for bx, by in placer.positions:
                if min_x <= bx <= max_x and min_y <= by <= max_y:
                    wt_objects.append(create_water_tower(bx, by, shaft_height, shaft_radius, tank_radius))
            if wt_objects:
                join_objects(wt_objects, 'WaterTowers')
                print("created {} water tower(s)".format(len(wt_objects)))
        except Exception as e:
            print("WARNING: water towers failed: " + str(e))

    # Terrain: drape the remaining ground features (roads, rails, water, towers) onto
    # the surface. LOD2 buildings were already seated during construction, so skip them.
    if terrain_sampler is not None:
        try:
            apply_terrain_to_objects(terrain_sampler, skip_buildings=(lod2_data is not None))
        except Exception as e:
            print("WARNING: terrain displacement failed: " + str(e))

    # Create the support base and optional borders
    mm_to_units = args.scale / 1000
    base_height = tc.BASE_HEIGHT_MM * mm_to_units * getattr(args, 'base_height_factor', 1.0)
    overlap = tc.BASE_OVERLAP_MM * mm_to_units
    base_bottom_z = -base_height + overlap

    if terrain_sampler is not None:
        # Build the terrain mesh at the elevation grid's resolution so a fine DGM1
        # grid (≈1 m) shows real detail; never coarser than the legacy 150 grid.
        terrain_grid = max(terrain_sampler.grid_w, terrain_sampler.grid_h, 150)
        terrain_obj = create_terrain_solid(terrain_sampler, min_x, min_y, max_x, max_y,
                                           base_bottom_z, overlap=overlap, grid_size=terrain_grid)
        terrain_obj.name = 'Base'
        base_cube = terrain_obj
        if not args.no_borders:
            add_borders(min_x, min_y, max_x, max_y,
                        tc.BORDER_WIDTH_MM * mm_to_units,
                        0, tc.BORDER_HEIGHT_MM * mm_to_units,
                        (tc.BUILDING_HEIGHT_MM + 1) * mm_to_units)
    else:
        base_cube = create_bounds(min_x, min_y, max_x, max_y, args.scale, args.no_borders,
                                  base_height_factor=getattr(args, 'base_height_factor', 1.0))

    # Add marker(s)
    if args.marker1 is not None:
        add_marker1(args, args.scale)

    return base_cube

def main():
    args = do_cmdline()
    remove_everything()

    for mesh_path in args.mesh_paths:
        import_mesh_file(mesh_path)

    if args.base_path:
        base_path = args.base_path
    else:
        base_path = os.path.splitext(args.mesh_paths[0])[0]
    if args.export_wireframe_png:
        export_wireframe_png(base_path, 'wireframe-flat', args.min_x, args.min_y, args.max_x, args.max_y)
    export_svg(base_path, args)
    base_cube = make_tactile_map(args)
    move_everything([-c for c in get_minimum_coordinate(base_cube)])
    if not args.no_stl_export:
        print_mesh_stats("before cleanup")
        cleanup_meshes_for_export()  # make manifold for OrcaSlicer / Bambu Studio
        print_mesh_stats("after cleanup")
        export_stl(base_path, args.scale)
        export_stl_separate(base_path, args.scale)
        if getattr(args, 'export_3mf', False):
            export_3mf(base_path, args.scale)
        export_blend_file(base_path)
    if args.export_wireframe_png:
        final_min_x, final_min_y, _final_min_z, final_max_x, final_max_y, _final_max_z = get_object_world_bounds(base_cube)
        export_wireframe_png(base_path, 'wireframe', final_min_x, final_min_y, final_max_x, final_max_y)
    bpy.ops.object.select_all(action='SELECT') # it's handy to have everything selected when getting into UI

if __name__ == "__main__":
    main()
