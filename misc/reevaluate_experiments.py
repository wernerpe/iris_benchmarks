import os
from iris_environments.environments import env_names, get_environment_builder, get_robot_instance_names
from pydrake.all import SceneGraphCollisionChecker,HPolyhedron, RandomGenerator
import pickle
from typing import List
import numpy as np
import shutil

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

experiment_name = 'default_experiments' #"fast_iris"
experiment_path = root + f"benchmarks/{experiment_name}" #root+f"/logs/{experiment_name}"
experiments_to_reevaluate = ['config_vfast', 'config_fast', 'config_precise']

def copy_directory(src, dest):
    try:
        shutil.copytree(src, dest)
        print(f"Directory copied from {src} to {dest}")
    except Exception as e:
        print(f"Error: {e}")

def evaluate_regions_ellipsoid(regions: List[HPolyhedron], 
                     col_checker: SceneGraphCollisionChecker, 
                     Ns = 20000):
    gen = RandomGenerator(1337)
    num_faces = [r.A().shape[0] for r in regions]
    fraction_in_collision = []
    for r in regions:
        samples = []
        prev = r.UniformSample(gen)
        for _ in range(Ns):
            prev = r.UniformSample(gen, prev, mixing_steps=50)
            samples.append(prev)
        col_free = col_checker.CheckConfigsCollisionFree(np.array(samples), parallelize=True)
        fraction_in_collision.append(np.sum(1-1.0*np.array(col_free))/Ns)
    print('calculating volumes')
    volumes = [r.MaximumVolumeInscribedEllipsoid().CalcVolume() for r in regions]
    return volumes, fraction_in_collision, num_faces 


for exp in experiments_to_reevaluate:
    reeval_dir = root+f"/logs/irisnp_reeval/{exp}"#root+f"/logs/{experiment_name}_reeval/{exp}"
    try:
        os.mkdir(reeval_dir)
    except:
        print('already created dir')
    exp_dir = root + f"/benchmarks/{experiment_name}/{exp}"#root + f'/logs/{experiment_name}/{exp}'
    copy_directory(exp_dir+'/parameters', reeval_dir+'/parameters')
    pkl_files = [e for e in os.listdir(exp_dir) if e[-3:]=='pkl']
    for pkl in pkl_files:
        env_name = pkl.split('_')[0]
        e_idx = env_names.index(env_name)
        print(f" pkl {pkl} env name {env_name}, env_idx {e_idx}")
        plant_builder = get_environment_builder(env_name)
        plant, scene_graph, diagram, diagram_context, plant_context, _ = plant_builder(usemeshcat=False)
        rob_names = get_robot_instance_names(env_name)
        robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
        checker = SceneGraphCollisionChecker(model = diagram.Clone(), 
                        robot_model_instances = robot_instances,
                        #configuration_distance_function = _configuration_distance,
                        edge_step_size = 0.125)

        with open(exp_dir+'/'+pkl, 'rb') as f:
            data = pickle.load(f)
        regions = data['regions']
        times = data['times']
        volumes, fraction_in_collision, num_faces = evaluate_regions_ellipsoid(regions, checker)
        results = {'regions': regions, 
                'times': times, 
                'volumes': volumes, 
                'fraction_in_collision': fraction_in_collision,
                'num_faces': num_faces}
        with open(reeval_dir+f'/{pkl}', 'wb') as f:
            pickle.dump(results, f)