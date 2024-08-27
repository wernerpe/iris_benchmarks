import os
#import collada
import numpy as np 
from get_AABB import get_AABB_cvxhull
import meshio
from stl import mesh

collision_geoms = {}
def rpy_to_matrix(vec):
    roll = vec[0]
    pitch = vec[1]
    yaw = vec[2]
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def read_obj(obj_filename):
    import trimesh

    # Load the .obj file
    mesh = trimesh.load_mesh(obj_filename)

    # Get vertex locations
    vertices = mesh.vertices
    geom_name = obj_filename.split('/')[-1].replace('.obj', '')
    aabb_min, aabb_max = get_AABB_cvxhull(np.array(vertices))
    size = aabb_max-aabb_min
    translation = (aabb_max+aabb_min)/2.0
    collision_geoms[geom_name.lower()] = [
                f'\t    <box size="{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}"/> \n', translation]
    
def convert_stl_to_obj(stl_filename, output_file):
    #not output_file.split('/')[-1] in os.listdir(os.path.dirname(stl_filename))
    if True:
        # Load the STL file
        stl_mesh = mesh.Mesh.from_file(stl_filename)

        # Get the base filename without extension
        base_filename = os.path.splitext(os.path.basename(stl_filename))[0]

        # Write the OBJ file
        vertices = []
        with open(output_file, 'w') as f:
            f.write("# OBJ file\n")
            for vertex in stl_mesh.vectors:
                f.write("v {:f} {:f} {:f}\n".format(*vertex[0]))
                f.write("v {:f} {:f} {:f}\n".format(*vertex[1]))
                f.write("v {:f} {:f} {:f}\n".format(*vertex[2]))
                f.write("f -3 -2 -1\n")
                vertices.append(vertex[0, :])
                vertices.append(vertex[1, :])
                vertices.append(vertex[2, :])

        geom_name = stl_filename.split('/')[-1].replace('.STL', '')
        aabb_min, aabb_max = get_AABB_cvxhull(np.array(vertices))
        size = aabb_max-aabb_min
        translation = (aabb_max+aabb_min)/2.0
        collision_geoms[geom_name.lower()] = [
                    f'\t    <box size="{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}"/> \n', translation]
        # "\t    <box>\n",
        # f"\t      <size>{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}</size>\n",
        # "\t    </box>\n",
        # ]
    print(f"Conversion successful. OBJ file saved at {output_file}")


import xml.etree.ElementTree as ET

def format_xml(input_file, output_file):
    # Parse the XML file
    tree = ET.parse(input_file)
    root = tree.getroot()
    lines = ET.tostring(root, encoding='unicode')
    
    
    # Open output file for writing
    with open(output_file, 'w') as f:
        # Start traversing the XML tree and format it
        f.write(lines)


path_gitfolder = os.path.dirname(os.path.abspath(__file__)) + "/.."
# Example usage:
input_file = path_gitfolder+"/iris_environments/assets/models/lbr_iiwa7_r800-urdf-package/urdf/lbr_iiwa7_r800.urdf"
output_file = path_gitfolder+"/iris_environments/assets/models/lbr_iiwa7_r800-urdf-package/urdf/lbr_iiwa7_r800_box_collision.urdf"
format_xml(input_file, output_file)
print(f"XML file '{input_file}' has been formatted and saved as '{output_file}'.")

package_name = "lbr_iiwa7_r800/"
package_path = path_gitfolder+"/iris_environments/assets/models/lbr_iiwa7_r800-urdf-package/"


def process_mesh_line(line):
    # Add your custom logic here

    print(f"Found mesh line: {line.strip()}")
    #get mesh path
    lineraw = line.strip()
    file_name = lineraw.split('"')[1]
    file_path = file_name.replace(f"package://{package_name}", package_path)
    print(f"New file path :{file_path}")
    if file_path[-4:] == '.STL':
        print('STL found')
        file_path_obj = file_path.replace('.STL', '.obj')
        convert_stl_to_obj(file_path, file_path_obj)
        print(file_path)
    if file_path[-4:] == '.obj':
        print('obj found')
        read_obj(file_path)
        print(file_path)

def traverse_file_and_process(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip(' ').startswith('<mesh filename'):
                process_mesh_line(line)

traverse_file_and_process(output_file)


lines = []
current_link_name = None
all_lines = []
with open(output_file, 'r') as file:
    for l in file:
        if not l.strip().startswith('<!---'):
            all_lines.append(l)
    
for l in all_lines:
    do_append=True
    
    if '<link name=' in l:
        current_link_name = l.replace('<link name="', '').replace('">', '').strip()
    if '<visual>' in l:
        l = l.replace('<visual>',f'<visual name="{current_link_name+"_visual"}">')
    if '<collision>' in l:
        l = l.replace('<collision>',f'<collision name="{current_link_name+"_collision"}">')
    if len(lines)>3 and '<collision' in lines[-2]:
        col_name = l.split('"')[1].split('/')[-1].replace('.STL', '').replace('.obj', '')
        l_col_geom = collision_geoms[col_name.lower()][0]
        lines.append(l_col_geom)
        do_append = False
    if len(lines)>3 and '<collision' in lines[-1] and 'origin' in l:
        origin_line = l
        do_append=False

    if len(lines)>3 and '</geometry' in lines[-1] and '<collision' in lines[-4]:
        do_append = False
        trans = collision_geoms[col_name.lower()][1]
        original_trans = [float(s) for s in origin_line.split('"')[1].replace('"', '').split(' ')[:3]]
        rpy = [float(s) for s in origin_line.split("rpy")[1].split('"')[1].split(' ') if s != '']
        t = (rpy_to_matrix(np.array(rpy))@np.array(trans).reshape(-1,1)).squeeze()
        lines.append(f'\t\t<origin xyz = "{t[0]+original_trans[0]} {t[1]+original_trans[1]} {t[2]+original_trans[2]}" rpy' + origin_line.split("rpy")[1])
        lines.append(l)
        #lines.append(lines[-6])
    if '.STL" />' in l:
        l = l.replace('.STL" />', '.obj" />') 
    if do_append:
            lines.append(l)
    print(len(lines))
    
path = output_file.replace('.urdf', '') + "_obj.urdf"
with open(path, 'w') as file:
    for l in lines:
        file.write(l)
