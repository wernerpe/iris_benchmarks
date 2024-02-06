import datetime
from pydrake.all import (HPolyhedron, 
                         VPolytope, 
                         RandomGenerator, 
                         SceneGraphCollisionChecker,
                         IrisInConfigurationSpace,
                         IrisOptions)
from typing import List
import numpy as np
import yaml

def run_default_settings(env_name):
    iris_options = IrisOptions()
    with open(f"default_options/{env_name}.yml") as f:
        settings = yaml.safe_load(f)

def run_custom_experiment(env_name, iris_handle):
    pass

def build_regions(seed_points : np.ndarray, iris_handle):
    regions = [iris_handle(p) for p in seed_points]
    return regions

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
        fraction_in_collision.append(np.sum(1-1.0*col_free)/Ns)
    return volumes, fraction_in_collision, num_faces 

def get_experiment_name(env_name, settings= 'default'):
    if settings != 'default':
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y%m%d%H%M%S")
        name = f"logs/{timestamp}_{env_name}_{settings}.pkl"
        return name
    else:
        return f"benchmarks/default_experiments/{env_name}.pkl"