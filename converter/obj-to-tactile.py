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

    def __init__(self, elevations, min_x, min_y, max_x, max_y):
        self.grid = elevations  # [row][col], row = Y (lat) bottom→top, col = X (lon) left→right
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.grid_h = len(elevations)
        self.grid_w = len(elevations[0]) if elevations else 0
        all_elev = [e for row in elevations for e in row]
        self.min_elev = min(all_elev) if all_elev else 0.0

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
        return e00*(1-tx)*(1-ty) + e10*tx*(1-ty) + e01*(1-tx)*ty + e11*tx*ty


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


def apply_terrain_to_objects(sampler):
    """Raise all map mesh vertices by terrain elevation at their (x, y) position."""
    for ob in list(bpy.context.scene.objects):
        if ob.type != 'MESH' or ob.name in ('TerrainSolid', 'Base', 'Borders', 'CornerInside', 'CornerTop'):
            continue
        mesh = ob.data
        # Objects have identity transform in this pipeline, so world coords == local coords
        for v in mesh.vertices:
            v.co.z += sampler.sample(v.co.x, v.co.y)
        mesh.update()
    bpy.context.view_layer.update()
    print("terrain displacement applied to all map objects")


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


def extrude_buildings_with_heights(joined_obj, height_lookup, default_height):
    """Split joined building mesh by connected components (via bmesh), extrude each to its OSM height."""
    parts = _split_mesh_by_components(joined_obj)
    print("separated {} building parts for individual extrusion".format(len(parts)))

    for ob in parts:
        bpy.context.view_layer.objects.active = ob
        bbox = ob.bound_box
        cx = (bbox[0][0] + bbox[6][0]) / 2 + ob.location.x
        cy = (bbox[0][1] + bbox[6][1]) / 2 + ob.location.y
        h = height_lookup.lookup(cx, cy)
        extrude_building(ob, h)
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
    parser.add_argument('--elevation-json', help='path to elevation grid JSON for terrain')
    parser.add_argument('--lon-min', type=float, help='map longitude minimum (for terrain)')
    parser.add_argument('--lon-max', type=float, help='map longitude maximum')
    parser.add_argument('--lat-min', type=float, help='map latitude minimum')
    parser.add_argument('--lat-max', type=float, help='map latitude maximum')
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
                                axis_forward='Y', axis_up='Z', global_scale=(1000 / scale))
    except AttributeError:
        # Blender 4.x: new operator with renamed parameters
        bpy.ops.wm.stl_export(filepath=stl_path, check_existing=False,
                              forward_axis='Y', up_axis='Z', global_scale=(1000 / scale))

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

def create_bounds(min_x, min_y, max_x, max_y, scale, no_borders):
    mm_to_units = scale / 1000
    if not no_borders:
        add_borders(min_x, min_y, max_x, max_y, tc.BORDER_WIDTH_MM * mm_to_units, \
                    0, tc.BORDER_HEIGHT_MM * mm_to_units, (tc.BUILDING_HEIGHT_MM + 1) * mm_to_units)
    base_height = tc.BASE_HEIGHT_MM * mm_to_units
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

def process_objects(min_x, min_y, max_x, max_y, scale, no_borders, building_height_lookup=None):
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
    
    # Buildings
    if joined_buildings:
        t = perf_clock()
        if building_height_lookup is not None:
            # Per-building heights: separate mesh by connected components, extrude each individually
            parts = extrude_buildings_with_heights(
                joined_buildings, building_height_lookup,
                default_height=tc.BUILDING_HEIGHT_MM * mm_to_units)
            if parts:
                join_objects(parts, 'Buildings')
            print("processed %d buildings with OSM heights in %.2f s" % (len(parts), perf_clock() - t))
        else:
            extrude_building(joined_buildings, tc.BUILDING_HEIGHT_MM * mm_to_units)
            fatten(joined_buildings)
            print("processing %d buildings took %.2f" % (len(buildings), perf_clock() - t))

    # Waters
    t = perf_clock()
    if len(waterways) > 0:
        joined_waterways = join_objects(waterways, 'JoinedWaterways')
        raise_ob(joined_waterways, tc.WATERWAY_DEPTH_MM * mm_to_units)
    if len(water_areas):
        for water in water_areas:
            water_wave_pattern(water, tc.WATER_AREA_DEPTH_MM * mm_to_units, scale)
        join_objects(water_areas, 'WaterAreas')
    print("processing waters took %.2f" % (perf_clock() - t))

    # Rails
    if joined_rails != None:
        do_ways(joined_rails, tc.ROAD_HEIGHT_CAR_MM * mm_to_units * 0.99, min_x, min_y, max_x, max_y) # 0.99 to avoid faces in the same coordinates with roads

    # Roads
    do_road_areas(joined_road_areas_car, tc.ROAD_HEIGHT_CAR_MM * mm_to_units)
    do_road_areas(joined_road_areas_ped, tc.ROAD_HEIGHT_PEDESTRIAN_MM * mm_to_units)
    do_ways(joined_roads_car, tc.ROAD_HEIGHT_CAR_MM * mm_to_units, min_x, min_y, max_x, max_y)
    do_ways(joined_roads_ped, tc.ROAD_HEIGHT_PEDESTRIAN_MM * mm_to_units, min_x, min_y, max_x, max_y)

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

    process_objects(min_x, min_y, max_x, max_y, args.scale, args.no_borders,
                    building_height_lookup=building_height_lookup)
    print("process_objects() took " + (str(perf_clock() - t)))

    # Terrain: apply elevation displacement after tactile processing
    terrain_sampler = None
    if elev_json and os.path.exists(elev_json):
        try:
            with open(elev_json, 'r', encoding='utf-8') as f:
                elev_data = json.load(f)
            terrain_sampler = TerrainSampler(
                elev_data['elevations'], min_x, min_y, max_x, max_y)
            print("terrain sampler ready, min_elev={:.1f}m, grid={}x{}".format(
                terrain_sampler.min_elev, terrain_sampler.grid_h, terrain_sampler.grid_w))
            apply_terrain_to_objects(terrain_sampler)
        except Exception as e:
            print("WARNING: terrain failed: " + str(e))
            terrain_sampler = None

    # Create the support base and optional borders
    mm_to_units = args.scale / 1000
    base_height = tc.BASE_HEIGHT_MM * mm_to_units
    overlap = tc.BASE_OVERLAP_MM * mm_to_units
    base_bottom_z = -base_height + overlap

    if terrain_sampler is not None:
        # Replace flat base cube with a solid terrain slab
        terrain_obj = create_terrain_solid(terrain_sampler, min_x, min_y, max_x, max_y,
                                           base_bottom_z, overlap=overlap, grid_size=150)
        terrain_obj.name = 'Base'
        base_cube = terrain_obj
        if not args.no_borders:
            add_borders(min_x, min_y, max_x, max_y,
                        tc.BORDER_WIDTH_MM * mm_to_units,
                        0, tc.BORDER_HEIGHT_MM * mm_to_units,
                        (tc.BUILDING_HEIGHT_MM + 1) * mm_to_units)
    else:
        base_cube = create_bounds(min_x, min_y, max_x, max_y, args.scale, args.no_borders)

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
        export_stl(base_path, args.scale)
        export_stl_separate(base_path, args.scale)
        export_blend_file(base_path)
    if args.export_wireframe_png:
        final_min_x, final_min_y, _final_min_z, final_max_x, final_max_y, _final_max_z = get_object_world_bounds(base_cube)
        export_wireframe_png(base_path, 'wireframe', final_min_x, final_min_y, final_max_x, final_max_y)
    bpy.ops.object.select_all(action='SELECT') # it's handy to have everything selected when getting into UI

if __name__ == "__main__":
    main()
