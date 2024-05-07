import datetime
from iris_environments.environments import get_environment_builder, get_robot_instance_names
from pydrake.all import (HPolyhedron, 
                         VPolytope, 
                         RandomGenerator, 
                         SceneGraphCollisionChecker,
                         IrisInConfigurationSpace,
                         IrisOptions,
                         MathematicalProgram,
                         OsqpSolver)
from typing import List
import numpy as np
import yaml
import time
import os
import pickle
import hashlib
import json

root = os.path.dirname(os.path.abspath(__file__)) 

def run_default_settings(env_name, config):
    iris_options = IrisOptions()
    
    configs= os.listdir(root+f"/default_experiments/{config}/parameters")
    for c in configs:
        if env_name in c:
            settings_path = root+f"/default_experiments/{config}/parameters/"+c
            old_hash = c.split('_')[-1][:-4]
            break
    num_trials = 1
    with open(settings_path, 'r') as f:
        settings = yaml.safe_load(f)
    for k in settings.keys():
        if k!='num_trials':
            setattr(iris_options, k, settings[k])
        else:
            num_trials = settings[k]

    settings_hash = hashlib.sha1(
                        json.dumps(settings, 
                            sort_keys=True)
                            .encode('utf-8')).hexdigest()[:10]
    
    are_settings_new = settings_hash != old_hash

    experiment_name = get_experiment_name(env_name, config, settings='default')
    if experiment_name.split('/')[-1]+f"_{settings_hash}.pkl" in \
            os.listdir(root+f"/default_experiments/{config}")\
        and not are_settings_new:
        path = root+f"/default_experiments/{config}/"+experiment_name.split('/')[-1]+f"_{settings_hash}.pkl"
        with open(path, 'rb') as f:
            results = pickle.load(f)
        return results, False, settings_hash

    #rename settings file 
    os.rename(settings_path, root+f"/default_experiments/{config}/parameters/{c.split('_')[0]}_{settings_hash}.yml")
    
    plant_builder = get_environment_builder(env_name)
    plant, scene_graph, diagram, diagram_context, plant_context, _ = plant_builder(usemeshcat=False)
    mut_cont = plant.GetMyMutableContextFromRoot(diagram_context)
    
    def iris_handle(pt, random_seed = iris_options.random_seed):
        iris_options.random_seed = random_seed
        plant.SetPositions(mut_cont, pt)
        return IrisInConfigurationSpace(plant, mut_cont, iris_options)
    
    rob_names = get_robot_instance_names(env_name)
    robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
    checker = SceneGraphCollisionChecker(model = diagram.Clone(), 
                    robot_model_instances = robot_instances,
                    #configuration_distance_function = _configuration_distance,
                    edge_step_size = 0.125)
    
    seed_points = load_seed_points(env_name)
    regions, times = build_regions(seed_points, 
                                   plant.GetPositionLowerLimits(), 
                                   plant.GetPositionUpperLimits(),
                                   iris_options.configuration_space_margin,
                                   iris_handle,
                                   num_trials,
                                   iris_options.random_seed)
    print("switching to evaluation")
    volumes, fraction_in_collision, num_faces = evaluate_regions(regions, checker)
    results = {'regions': regions, 
               'times': times, 
               'volumes': volumes, 
               'fraction_in_collision': fraction_in_collision,
               'num_faces': num_faces}
    return results, True, settings_hash

def run_custom_experiment(env_name, 
                          plant, 
                          diagram, 
                          iris_handle, 
                          configuration_space_margin,
                          random_seed,
                          num_trials):
    assert num_trials > 0

    seed_points = load_seed_points(env_name)
    regions, times = build_regions(seed_points, 
                    plant.GetPositionLowerLimits(), 
                    plant.GetPositionUpperLimits(),
                    configuration_space_margin,
                    iris_handle,
                    num_trials,
                    random_seed)
    
    rob_names = get_robot_instance_names(env_name)
    robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
    checker = SceneGraphCollisionChecker(model = diagram.Clone(), 
                    robot_model_instances = robot_instances,
                    #configuration_distance_function = _configuration_distance,
                    edge_step_size = 0.125)
    
    volumes, fraction_in_collision, num_faces = evaluate_regions(regions, checker)
    results = {'regions': regions, 
               'times': times, 
               'volumes': volumes, 
               'fraction_in_collision': fraction_in_collision,
               'num_faces': num_faces}
    return results


def project_to_polytope(pt: np.ndarray,
                        P: HPolyhedron,
                        lower_limits: np.ndarray,
                        upper_limits: np.ndarray) -> np.ndarray:
    assert(len(upper_limits.squeeze()) == P.ambient_dimension())
    prog = MathematicalProgram()
    s = prog.NewContinuousVariables(len(upper_limits))
    #prog.AddBoundingBoxConstraint(lower_limits, upper_limits, s)
    prog.AddLinearConstraint(P.A(), np.full_like(P.b(), -np.inf), P.b(), s)
    prog.AddQuadraticErrorCost(np.eye(s.shape[0]), pt, s)
    osqp_solver = OsqpSolver()
    result = osqp_solver.Solve(prog)
    return result.GetSolution(s)

def build_regions(seed_points : np.ndarray,
                  lower_limits: np.ndarray,
                  upper_limits: np.ndarray,
                  cspace_margin: float, 
                  iris_handle,
                  num_trials: int,
                  random_seed: int):
    
    assert num_trials>0
    if num_trials>1:
        np.random.seed(random_seed)
    regions = []
    times = []
    domain = HPolyhedron.MakeBox(lower_limits+cspace_margin, upper_limits-cspace_margin)
    for i, p in enumerate(seed_points):
        for _ in range(num_trials):
            if num_trials>1:
                #set random seed for iris for repeated trials
                rand_seed_trial = np.random.randint(20000)
            else:
                rand_seed_trial = random_seed
            current_time = datetime.datetime.now()
            timestamp = current_time.strftime("[%H_%M_%S]")
            print(timestamp)
            p_proj = project_to_polytope(p, domain, lower_limits, upper_limits)
            assert domain.PointInSet(p_proj)
            t1 = time.time()
            r = iris_handle(p_proj, rand_seed_trial)
            t2 = time.time()
            times.append(t2-t1)
            regions.append(r)
        print(f"Seedpoint {i+1}/{len(seed_points)}")
    return regions, times

def evaluate_regions(regions: List[HPolyhedron], 
                     col_checker: SceneGraphCollisionChecker, 
                     Ns = 5000):
    gen = RandomGenerator(1337)
    num_faces = [r.A().shape[0] for r in regions]
    fraction_in_collision = []
    for r in regions:
        samples = []
        prev = r.UniformSample(gen)
        for _ in range(Ns):
            prev = r.UniformSample(gen, prev, mixing_steps=10)
            samples.append(prev)
        col_free = col_checker.CheckConfigsCollisionFree(np.array(samples), parallelize=True)
        fraction_in_collision.append(np.sum(1-1.0*np.array(col_free))/Ns)
    print('calculating volumes')
    if regions[0].ambient_dimension() >=14:
        volumes = [r.MaximumVolumeInscribedEllipsoid().CalcVolume() for r in regions]
    else:
        volumes = [r.CalcVolumeViaSampling(gen, 0.001, int(1e7)).volume for r in regions]
    return volumes, fraction_in_collision, num_faces 

def get_experiment_name(env_name, config=None, settings= 'default'):
    if settings != 'default':
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y%m%d%H%M%S")
        name = f"logs/{timestamp}_{env_name}_{settings}"
        return name
    else:
        assert config is not None
        return root + f"/default_experiments/{config}/{env_name}"
    
def load_seed_points(env_name):
    seed_point_file = root+'/seedpoints/'+env_name+'.yml'
    with open(seed_point_file, 'r') as f:
        seed_points = yaml.safe_load(f)
    return np.array(seed_points['seedpoints'])
    