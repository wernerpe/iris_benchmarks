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

root = os.path.dirname(os.path.abspath(__file__)) 

def run_default_settings(env_name):
    iris_options = IrisOptions()
    
    with open(root+f"/default_options/{env_name}.yml", 'r') as f:
        settings = yaml.safe_load(f)
    for k in settings.keys():
        setattr(iris_options, k, settings[k])

    experiment_name = get_experiment_name(env_name, settings='default')
    if experiment_name.split('/')[-1] in os.listdir(root+"/default_experiments"):
        with open(experiment_name, 'rb') as f:
            results = pickle.load(f)
        return results, False

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
    return results, True

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
        name = f"logs/{timestamp}_{env_name}_{settings}.pkl"
        return name
    else:
        return root + f"/default_experiments/{env_name}.pkl"
    
def load_seed_points(env_name):
    seed_point_file = root+'/seedpoints/'+env_name+'.yml'
    with open(seed_point_file, 'r') as f:
        seed_points = yaml.safe_load(f)
    return np.array(seed_points['seedpoints'])
    