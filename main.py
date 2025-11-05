
import bpy
import os
import sys
import importlib
import math
import range_scanner 
from mathutils import Vector

#Defining and registering the script directory
script_dir = os.path.dirname(bpy.data.filepath)
if script_dir not in sys.path:
    sys.path.append(script_dir)
print(script_dir)

import blender_utils
import urdf_utils
import lidar_utils

importlib.reload(blender_utils)
importlib.reload(urdf_utils)
importlib.reload(lidar_utils)

# Define all file paths ---
TUG_URDF = os.path.join(script_dir, "Tugs", "t5.urdf")
TUG_STL  = os.path.join(script_dir, "Tugs", "t5.ply")
AC_URDF  = os.path.join(script_dir, "AC", "a320_ceo.urdf")
AC_STL   = os.path.join(script_dir, "AC", "a320_ceo.ply")
EXPORT_DIR = os.path.join(script_dir, "Outputs")
os.makedirs(EXPORT_DIR, exist_ok=True)


#  Define the save/load path for your scene ---
BLEND_FILE_PATH = os.path.join(script_dir, "scene4.blend")


# Set this to True to load the BLEND_FILE_PATH,Set this to False to create a new scene from scratch.
LOAD_FROM_BLEND_FILE = True


print("STARTING SCRIPT")
scene_was_loaded = False

if LOAD_FROM_BLEND_FILE:
    scene_was_loaded = blender_utils.load_blend_file(BLEND_FILE_PATH)
    if not scene_was_loaded:
        print("Load failed. Halting script.")
else:
    #Creating a new scene from scratch.
    blender_utils.clear_scene()
    blender_utils.create_boundary_walls()

    # Load Tug 
    tug_start_location = (0, 0, 0)
    tug_object = blender_utils.import_ply(TUG_STL, "Tug_t5")
    blender_utils.assign_material(tug_object.name, color=(0.8, 0.1, 0.1, 1.0))
    blender_utils.set_position(tug_object.name, tug_start_location)
    tug_data = urdf_utils.load_and_verify_urdf(TUG_URDF)
    tug_world_poses = urdf_utils.link_positions(tug_data, base_position=tug_start_location)

    # Load Aircraft (AC) 
    ac_start_location = (0, 0, 0) 
    ac_object = blender_utils.import_ply(AC_STL, "a3320_ceo")
    blender_utils.assign_material(ac_object.name, color=(0.9, 0.9, 0.9, 1.0))
    blender_utils.set_position(ac_object.name, ac_start_location)
    ac_data = urdf_utils.load_and_verify_urdf(AC_URDF)
    ac_world_poses = urdf_utils.link_positions(ac_data, base_position=ac_start_location)

    # Align AC to Tug 
    caster_pos = urdf_utils.get_link_position(tug_world_poses, "caster")
    towbar_pos = urdf_utils.get_link_position(ac_world_poses, "towbar")

    if caster_pos is not None and towbar_pos is not None:
        new_ac_pos = ac_start_location + (caster_pos - towbar_pos)
        blender_utils.set_position(ac_object.name, new_ac_pos)
        ac_world_poses = urdf_utils.link_positions(ac_data, base_position=new_ac_pos)
    else:
        print("Error: Could not find 'caster' or 'towbar' link for alignment.")

    # Create Debug Markers ---
    #urdf_utils.draw_link_markers(tug_world_poses, size=0.03)



    # Saving the new scene in blender
    print(f"Scene creation complete. Saving to: {BLEND_FILE_PATH}")
    blender_utils.save_blend_file(BLEND_FILE_PATH)

# ============================================================================================================================
# Setup Lidar

if not LOAD_FROM_BLEND_FILE or scene_was_loaded:
    
    lidar_utils.enable_scanner_addon()

    # We find our permanent objects
    tug_obj = bpy.data.objects.get("Tug_t5")
    ac_obj = bpy.data.objects.get("a3320_ceo")

    # Check if tug was found
    if tug_obj and ac_obj:
        print("Found Tug and AC, setting up dynamic Lidar...")
        
        # Get Tug's and AC's kinematics at its CURRENT position
        tug_data = urdf_utils.load_and_verify_urdf(TUG_URDF)
        tug_current_location = tug_obj.location 
        tug_world_poses = urdf_utils.link_positions(tug_data, base_position=tug_current_location)

        ac_data = urdf_utils.load_and_verify_urdf(AC_URDF)

        #angles you want the tug rotation in z axis
        TUG_ORIENTATIONS = [45]
        SURFACES = ("Cube", "Cube.001") #name of the mesh used for sampling lidar positions
        for name in SURFACES:
            obj = bpy.data.objects.get(name)
            
            if obj is None:
                print(f"Surface '{name}' not found, skipping.")
                continue
            if obj.type != 'MESH':
                print(f"Object '{name}' is not a mesh, skipping.")
                continue

            # force dependency graph update to ensure valid bound_box
            bpy.context.view_layer.update()
            obj.parent = tug_obj

            print(f"Current location of {name}: {obj.location}")
            print(f"Current box for {name}: {list(obj.bound_box)}")

        for yaw_deg in TUG_ORIENTATIONS:
            yaw_rad = math.radians(yaw_deg)
            tug_obj.rotation_euler = (0.0, 0.0, yaw_rad)
            bpy.context.view_layer.update() 

            #Realignment of the Tug with AC model..
            tug_world_poses = urdf_utils.link_positions(tug_data, base_position=tug_obj.location)
            caster_pos = urdf_utils.get_link_position(tug_world_poses, "caster")
            print(caster_pos)
            ac_world_poses = urdf_utils.link_positions(ac_data, base_position=ac_obj.location)
            towbar_pos = urdf_utils.get_link_position(ac_world_poses, "towbar")
            print(towbar_pos)
            if caster_pos is not None and towbar_pos is not None:
                correction = caster_pos - towbar_pos
                correction_vector = Vector(correction)
                print(correction_vector)
                new_ac_pos = ac_obj.location + correction_vector
                blender_utils.set_position(ac_obj.name, new_ac_pos)
                bpy.context.view_layer.update()
            else : 
                print("Not able to perform realignment")

            print(f"\n=== Scanning with Tug orientation {yaw_deg}° ===")

            tug_world_poses = urdf_utils.link_positions(tug_data, base_position=tug_obj.location)

            
            grid_points = [(s, p) for s in SURFACES for p in blender_utils.get_grid_points(s)[0]]
          

    
            # Define all Lidar properties
            lidar_name = "lidar"
            lidar_scale = (0.15, 0.15, 0.15)
            lidar_rotation = (90, 0, 90)
        
            #CREATE LIDAR (i.e camera)
            lidar_cam = blender_utils.create_camera(
                name=lidar_name,
                location=grid_points[0][1],
                rotation_degrees=lidar_rotation,
                scale=lidar_scale
            )
            
            z_offset = 0.1   # raise LiDAR a bit above the plate (meters)
            show_wire = True  # draw scans as wire so point clouds are easy to see
            orientation_tag = f"yaw_{yaw_deg:03d}"
            print("Ahya poicha")

            for idx, (surf,pt) in enumerate(grid_points, start=1):
                # move lidar to this grid point (+ small Z )
                print(f"[SCAN] {orientation_tag} | {surf} | {idx}/{len(grid_points)} "f"at {(round(pt.x,3), round(pt.y,3), round(pt.z,3))}")
                pos = Vector((pt.x, pt.y, pt.z + z_offset))
                blender_utils.set_position(lidar_cam.name, pos)
                print("Ahya poicha3")
                #bpy.context.view_layer.update()
                out_name = f"{orientation_tag}_{surf}_scan_{idx:03d}"
                
                # remember current objects to detect what the scanner adds
                before = {o.name for o in bpy.data.objects}
                print("Ahya poicha")
                lidar_utils.run_detailed_rotating_scan(
                    scanner_name=lidar_cam.name,
                    output_dir=EXPORT_DIR,
                    output_filename=out_name,
                    export=True,          # set True later if you want files
                    add_mesh_to_scene=True 
                    #target_object=bpy.data.objects.get(surf)   
                )
                bpy.context.view_layer.update()
                            

    else:
        print("ERROR in Action Block: Could not find required objects.")
        print(f"  Found Tug: {'Yes' if tug_obj else 'No'}")
        print(f"  Found AC: {'Yes' if ac_obj else 'No'}")

print("\n--- SCRIPT COMPLETE ---")

