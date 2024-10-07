#path_mesh_folder = '/home/peter/gitcspace/mmt_gcs/assets/kinova/kortex_description/arms/gen3/7dof/meshes/'
path_mesh_folder = '/home/peter/gitcspace/iris_benchmarks/iris_environments/assets/models/triangle_env_meshes'#/triangle_and_hole.STL'

import os
import meshio
from stl import mesh

# def convert_stl_to_obj(stl_filename, output_folder):
#     # Load the STL file
#     stl_mesh = mesh.Mesh.from_file(stl_filename)

#     # Create the output folder if it doesn't exist
#     if not os.path.exists(output_folder):
#         os.makedirs(output_folder)

#     # Get the base filename without extension
#     base_filename = os.path.splitext(os.path.basename(stl_filename))[0]

#     # Construct the output filename
#     obj_filename = os.path.join(output_folder, base_filename + ".obj")

#     # Convert to meshio format
#     points = stl_mesh.points.reshape((-1, 3))
#     cells = [("triangle", stl_mesh.vectors)]
#     meshio.write_points_cells(obj_filename, points, cells, file_format="obj")

def convert_stl_to_obj(stl_filename, output_folder):
    # Load the STL file
    stl_mesh = mesh.Mesh.from_file(stl_filename)

    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Get the base filename without extension
    base_filename = os.path.splitext(os.path.basename(stl_filename))[0]

    # Construct the output filename
    obj_filename = os.path.join(output_folder, base_filename + ".obj")

    # Write the OBJ file
    with open(obj_filename, 'w') as f:
        f.write("# OBJ file\n")
        for vertex in stl_mesh.vectors:
            f.write("v {:f} {:f} {:f}\n".format(*vertex[0]))
            f.write("v {:f} {:f} {:f}\n".format(*vertex[1]))
            f.write("v {:f} {:f} {:f}\n".format(*vertex[2]))
            f.write("f -3 -2 -1\n")
            
def convert_all_stl_to_obj(input_folder, output_folder):
    # List all STL files in the input folder
    stl_files = [f for f in os.listdir(input_folder) if f.endswith(".STL")]

    # Convert each STL file to OBJ
    for stl_file in stl_files:
        stl_path = os.path.join(input_folder, stl_file)
        convert_stl_to_obj(stl_path, output_folder)

# Example usage:
input_folder = path_mesh_folder
output_folder = path_mesh_folder
convert_all_stl_to_obj(input_folder, output_folder)
