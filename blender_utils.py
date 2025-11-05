
import bpy
import math
import bmesh
import os
from mathutils import Vector, Euler
import random



def clear_scene():
    """Deletes everything from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    if bpy.context.selected_objects:
        bpy.ops.object.delete(use_global=False, confirm=False)

def create_boundary_walls(size=60, distance=30):
    """Creates a simple floor and 3 walls."""
    wall_materials = bpy.data.materials.new(name="WallMaterial")
    wall_materials.use_nodes = True
    wall_materials.node_tree.nodes["Principled BSDF"].inputs['Base Color'].default_value = (0.2, 0.2, 0.2, 1)

    walls_to_create = [
        ("Wall_Back", (0, -distance, size / 2), (math.radians(90), 0, 0)),
        ("Wall_Left", (-distance , 0, size / 2), (0, math.radians(90), 0)),
        ("Wall_Right", ( 0,distance , size / 2), (math.radians(90), 0, 0)),
        ("Floor", (0, 0, -0.6), (0, 0, 0))
    ]

    for name, loc, rot in walls_to_create:
        bpy.ops.mesh.primitive_plane_add(
            size=size,
            enter_editmode=False,
            align='WORLD',
            location=loc,
            rotation=rot
        )
        wall = bpy.context.active_object
        wall.name = name
        wall.data.materials.append(wall_materials)
        print(f"Created '{name}'")

def import_stl(filepath, model_name):
    """Imports an STL model and gives it a specific name."""
    bpy.ops.wm.stl_import(filepath=filepath)
    imported_obj = bpy.context.active_object
    imported_obj.name = model_name
    print(f"Imported {model_name}")
    return imported_obj  

def import_ply(filepath, model_name):
    """Imports an ply model and gives it a specific name."""
    bpy.ops.wm.ply_import(filepath=filepath)
    imported_obj = bpy.context.active_object
    imported_obj.name = model_name
    print(f"Imported {model_name}")
    return imported_obj  

def get_position(object_name):
    """Gets the world-space location of an object."""
    obj = bpy.data.objects.get(object_name)
    if obj:
        return obj.location
    else:
        print(f"Object {object_name} not found.")
        return None

def set_position(object_name, position):
    """Moves the object to the desired world-space position."""
    obj = bpy.data.objects.get(object_name)
    if obj:
        obj.location = position
        print(f"Set position of '{object_name}' to {position}")
    else:
        print(f"Warning: Object '{object_name}' not found.")  

def set_transform(object_name, position, rotation):
    """Sets the object's location and rotation (in degrees)."""
    obj = bpy.data.objects.get(object_name)
    if not obj:
        print(f"Warning: Object '{object_name}' not found.")
        return
        
    rotation_radians = (
        math.radians(rotation[0]),
        math.radians(rotation[1]),
        math.radians(rotation[2])
    )
    
    obj.location = position
    obj.rotation_euler = rotation_radians
    
    print(f"Set transform for '{object_name}':")
    print(f"  Position: {position}")
    print(f"  Rotation (XYZ degrees): {rotation}")

def assign_material(object_name, color=(0.6, 0.6, 0.6, 1.0), roughness=0.5):
    """Assigns a simple new material to an object for range_scanner addon"""
    obj = bpy.data.objects.get(object_name)
    if obj:
        mat_name = f"Mat_{object_name}"
        material = bpy.data.materials.new(name=mat_name)
        material.use_nodes = True
        shader_node = material.node_tree.nodes["Principled BSDF"]
        shader_node.inputs['Base Color'].default_value = color
        shader_node.inputs['Roughness'].default_value = roughness

        if obj.data.materials:
            obj.data.materials[0] = material
        else:
            obj.data.materials.append(material)
        print(f"Assigned basic material '{mat_name}' to '{object_name}'.")

def make_object_scannable(object_name):
    """Ensures an object is visible to the Lidar scanner."""
    obj = bpy.data.objects.get(object_name)
    if obj:
        obj.hide_render = False
        obj.cycles_visibility.camera = True
        obj.cycles_visibility.diffuse = True
        obj.cycles_visibility.shadow = True
        print(f" Made '{object_name}' fully visible for scanning.")
    else:
        print(f"Warning: Could not find '{object_name}' to make it scannable.")

# def create_debug_marker(position, name="DebugMarker", size=0.01):
#     """
#     Creates a small red sphere at a given position for debugging,
#     using the stable bpy.data API.
#     """
    
#     # Create the mesh and object ---
#     mesh = bpy.data.meshes.new(name)
#     marker = bpy.data.objects.new(name, mesh)
    
#     #  Link object to the scene ---
#     bpy.context.scene.collection.objects.link(marker)
    
#     #  Create the sphere geometry using bmesh ---
#     bm = bmesh.new()
#     bmesh.ops.create_uvsphere(
#         bm,
#         u_segments=16, # Low-poly sphere
#         v_segments=8,
#         radius=1.0 # We will scale this down
#     )
#     bm.to_mesh(mesh) # Write the bmesh data to the object's mesh
#     bm.free() # Free the bmesh memory
    
#     #  Set location and scale ---
#     marker.location = position
#     marker.scale = (size, size, size) # Scale the 1-unit sphere down
    
#     #  Material Handling (this code is unchanged) ---
#     mat_name = "RedMarkerMaterial"
#     if mat_name in bpy.data.materials:
#         red_mat = bpy.data.materials[mat_name]
#     else:
#         red_mat = bpy.data.materials.new(name=mat_name)
#         red_mat.use_nodes = True
#         bsdf = red_mat.node_tree.nodes["Principled BSDF"]
#         bsdf.inputs['Base Color'].default_value = (1.0, 0.0, 0.0, 1.0) # Red
#         bsdf.inputs['Roughness'].default_value = 0.5

#     if marker.data.materials:
#         marker.data.materials[0] = red_mat
#     else:
#         marker.data.materials.append(red_mat)

def save_blend_file(filepath):
    """Saves the current Blender scene to a .blend file."""
    if not filepath.endswith(".blend"):
        filepath += ".blend"
    
    bpy.ops.wm.save_as_mainfile(filepath=filepath)
    print(f"Scene saved to: {filepath}")

def load_blend_file(filepath):
    """Loads a .blend file, replacing the current scene."""
    if not os.path.exists(filepath):
        print(f"ERROR: File not found, cannot load: {filepath}")
        return False
        
    bpy.ops.wm.open_mainfile(filepath=filepath)
    print(f"Scene loaded from: {filepath}")
    return True

def create_camera(name, location, rotation_degrees, scale=(1,1,1)):
    """
    Creates a new camera object in the scene.
    
    """
    try:
        #Create new camera data
        cam_data = bpy.data.cameras.new(name=name)
        
        # Create new object with that data
        cam_obj = bpy.data.objects.new(name, cam_data)
        
        # Link the object to the scene's main collection
        bpy.context.scene.collection.objects.link(cam_obj)
        
        # Set all its transform properties
        cam_obj.location = location
        cam_obj.scale = scale
        cam_obj.rotation_euler = (
            math.radians(rotation_degrees[0]),
            math.radians(rotation_degrees[1]),
            math.radians(rotation_degrees[2])
        )
        
        print(f" Successfully created camera '{name}' at {location}")
        return cam_obj
        
    except Exception as e:
        print(f"ERROR in create_camera: {e}")
        return None

###############################################################################################################################################################

def world_box(obj):
    """This function identifies the plate or cube mesh added for the placement of lidar and convert its coordintes in reference to world coordinate frame."""
    M = obj.matrix_world
    corners = [M @ Vector(corner) for corner in obj.bound_box]
    c = Vector((0,0,0))
    for v in corners:
        c += v
    c /= 8.0
    return corners, c


def get_plate_area(obj_name):
    """
    Returns the top surface area of a plate in world space.    """

    # Get the object from Blender
    obj = bpy.data.objects.get(obj_name)
    if not obj or obj.type != 'MESH':
        print(f"Error: {obj_name} not found or not a mesh.")
        return 0.0

    # Get its evaluated mesh (includes transformations)
    deps = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(deps)
    mesh = obj_eval.to_mesh()
    M = obj_eval.matrix_world
    Z = Vector((0, 0, 1))  # “Up” direction in world space

    #  compute area of any polygon (in world space)
    def polygon_area(poly):
        verts = [M @ mesh.vertices[i].co for i in poly.vertices]
        if len(verts) < 3:
            return 0.0
        area = 0.0
        for i in range(1, len(verts) - 1):
            area += 0.5 * ((verts[i] - verts[0]).cross(verts[i + 1] - verts[0])).length
        return area

    # For a single plane (1 face) → just return its area
    if len(mesh.polygons) == 1:
        return polygon_area(mesh.polygons[0])

    #  Otherwise (for boxes): find faces facing upwards
    top_faces = []
    top_z = -1e9
    for p in mesh.polygons:
        normal_world = (M.to_3x3() @ p.normal).normalized()
        if normal_world.dot(Z) > 0.9:  # nearly facing up
            z = (M @ p.center).z
            if abs(z - top_z) < 1e-6:
                top_faces.append(p)
            elif z > top_z:
                top_z = z
                top_faces = [p]

    # Add up all top-face areas
    total_area = sum(polygon_area(p) for p in top_faces)
    return total_area

def get_grid_points(plate_name: str,
                          nx: int = 1,
                          ny: int = 2,
                          margin: float = 0.10,
                          height_offset: float = 0.03,
                          visualize: bool = False,
                          coll_name: str = "Grid_Debug",
                          empty_size: float = 0.04):
    """
    Return evenly spaced world-space points on the TOP face of a mesh. By chanmging the nx and ny values we can increase or decrease the no. of locations..
    """
    obj = bpy.data.objects.get(plate_name)
    if not obj or obj.type != 'MESH':
        raise ValueError(f"{plate_name} not found or not a mesh.")

    # Find the top face as a quad in WORLD space using the bbox 
    M = obj.matrix_world
    mesh = obj.data
    if len(mesh.polygons) == 1 and len(mesh.polygons[0].vertices) == 4:
        #Single face (like a UI Plane)
        poly = mesh.polygons[0]
        top_world = [M @ mesh.vertices[i].co for i in poly.vertices]

    else:
        # Fallback for solids or boxes: use bounding box
        bb_local = list(obj.bound_box)
        zmax = max(p[2] for p in bb_local)
        top_local = [Vector(p) for p in bb_local if abs(p[2] - zmax) < 1e-9]
        if len(top_local) >= 4:
        # Some shapes duplicate corners — take unique ones
            unique = []
            for v in top_local:
                if not any((v - u).length < 1e-6 for u in unique):
                    unique.append(v)
                top_world = [M @ v for v in unique[:4]]
        else:
            raise RuntimeError(f"{plate_name}: could not determine top face.")


    c = sum(top_world, Vector((0,0,0))) / 4.0
    def ang(pt): return (pt - c).to_2d().angle_signed(Vector((1,0)))
    top_world.sort(key=ang)
    p0, p1, p2, p3 = top_world

    # Plate normal (world)
    z_hat = (p1 - p0).cross(p3 - p0).normalized()
    if z_hat.length == 0:
        raise RuntimeError("Degenerate plate: cannot compute normal.")

    # Bilinear interpolation inside the quad with margins
    u0, u1 = margin, 1.0 - margin
    v0, v1 = margin, 1.0 - margin
    us = [u0 + i*(u1-u0)/(nx-1) for i in range(nx)] if nx > 1 else [(u0+u1)/2]
    vs = [v0 + j*(v1-v0)/(ny-1) for j in range(ny)] if ny > 1 else [(v0+v1)/2]

    def lerp(a,b,t): return a + t*(b-a)
    def bilinear(u,v):
        a = lerp(p0, p1, u)
        b = lerp(p3, p2, u)
        return lerp(a, b, v)

    points, indices = [], []
    for j, v in enumerate(vs):
        for i, u in enumerate(us):
            P = bilinear(u, v) + z_hat * height_offset
            points.append(P)
            indices.append((i, j))

    # Optional visualization as empties
    if visualize:
        coll = bpy.data.collections.get(coll_name) or bpy.data.collections.new(coll_name)
        if coll.name not in bpy.context.scene.collection.children:
            bpy.context.scene.collection.children.link(coll)
        for o in list(coll.objects):  # clear previous markers
            coll.objects.unlink(o)
        for k, P in enumerate(points):
            e = bpy.data.objects.new(f"grid_{k:03d}", None)
            e.empty_display_type = 'PLAIN_AXES'
            e.empty_display_size = empty_size
            e.location = P
            coll.objects.link(e)

    return points, indices


