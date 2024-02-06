import datetime
from iris_environments.environments import get_environment_builder, get_robot_instance_names
from pydrake.all import (HPolyhedron, 
                         VPolytope, 
                         RandomGenerator, 
                         SceneGraphCollisionChecker,
                         IrisInConfigurationSpace,
                         IrisOptions)
from typing import List
import numpy as np
import yaml
import time
import os
import pickle
import hashlib
import json

root = os.path.dirname(os.path.abspath(__file__)) 

def run_default_settings(env_name):
    iris_options = IrisOptions()
    
    configs= os.listdir(root+f"/default_options")
    for c in configs:
        if env_name in c:
            settings_path = root+f"/default_options/"+c
            old_hash = c.split('_')[-1][:-4]
            break
    with open(settings_path, 'r') as f:
        settings = yaml.safe_load(f)
    for k in settings.keys():
        setattr(iris_options, k, settings[k])

    settings_hash = hashlib.sha1(
                        json.dumps(settings, 
                            sort_keys=True)
                            .encode('utf-8')).hexdigest()[:10]
    
    are_settings_new = settings_hash != old_hash

    experiment_name = get_experiment_name(env_name, settings='default')
    if experiment_name.split('/')[-1]+f"_{settings_hash}.pkl" in \
            os.listdir(root+"/default_experiments")\
        and not are_settings_new:
        path = root+"/default_experiments/"+experiment_name.split('/')[-1]+f"_{settings_hash}.pkl"
        with open(path, 'rb') as f:
            results = pickle.load(f)
        return results, False, settings_hash

    #rename settings file 
    os.rename(settings_path, root+f"/default_options/{c.split('_')[0]}_{settings_hash}.yml")
    
    plant_builder = get_environment_builder(env_name)
    plant, scene_graph, diagram, diagram_context, plant_context, _ = plant_builder(usemeshcat=False)
    mut_cont = plant.GetMyMutableContextFromRoot(diagram_context)
    
    def iris_handle(pt):
        plant.SetPositions(mut_cont, pt)
        return IrisInConfigurationSpace(plant, mut_cont, iris_options)
    
    rob_names = get_robot_instance_names(env_name)
    robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
    checker = SceneGraphCollisionChecker(model = diagram.Clone(), 
                    robot_model_instances = robot_instances,
                    #configuration_distance_function = _configuration_distance,
                    edge_step_size = 0.125)
    
    seed_points = load_seed_points(env_name)
    regions, times = build_regions(seed_points, iris_handle)
    volumes, fraction_in_collision, num_faces = evaluate_regions(regions, checker)
    results = {'regions': regions, 
               'times': times, 
               'volumes': volumes, 
               'fraction_in_collision': fraction_in_collision,
               'num_faces': num_faces}
    return results, True, settings_hash

def run_custom_experiment(env_name, iris_handle):
    pass

def build_regions(seed_points : np.ndarray, iris_handle):
    regions = []
    times = []
    for p in seed_points:
        t1 = time.time()
        r = iris_handle(p)
        t2 = time.time()
        times.append(t2-t1)
        regions.append(r)
    return regions, times

def evaluate_regions(regions: List[HPolyhedron], 
                     col_checker: SceneGraphCollisionChecker, 
                     Ns = 5000):
    gen = RandomGenerator(1337)
    num_faces = [r.A().shape[0] for r in regions]
    volumes = [VPolytope(r).CalcVolume() for r in regions]
    fraction_in_collision = []
    for r in regions:
        samples = []
        prev = r.UniformSample(gen)
        for _ in range(Ns):
            prev = r.UniformSample(gen, prev, mixing_steps=10)
            samples.append(prev)
        col_free = col_checker.CheckConfigsCollisionFree(np.array(samples), parallelize=True)
        fraction_in_collision.append(np.sum(1-1.0*np.array(col_free))/Ns)
    return volumes, fraction_in_collision, num_faces 

def get_experiment_name(env_name, settings= 'default'):
    if settings != 'default':
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y%m%d%H%M%S")
        name = f"logs/{timestamp}_{env_name}_{settings}"
        return name
    else:
        return root + f"/default_experiments/{env_name}"
    
def load_seed_points(env_name):
    seed_point_file = root+'/seedpoints/'+env_name+'.yml'
    with open(seed_point_file, 'r') as f:
        seed_points = yaml.safe_load(f)
    return np.array(seed_points['seedpoints'])
    