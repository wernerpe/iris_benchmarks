import sys
sys.path.insert(0,'/home/rebecca/Documents/code/fork_drake/drake-build/install/lib/python3.10/site-packages')


import hashlib
import json
import yaml
from pydrake.all import (IrisInConfigurationSpace,
                        #  RayIris,
                         IrisOptions,
                         SceneGraphCollisionChecker,
                         HPolyhedron,
                         Hyperellipsoid, IrisNp2Options, IrisNp2SamplingStrategy, IrisNp2)
from iris_environments.environments import get_robot_instance_names
import os

def get_iris_handle(env_name,
                    settings_name, 
                    plant, 
                    diagram, 
                    diagram_context):
    root = os.path.dirname(os.path.abspath(__file__)) 
    settings_path = root + f"/{settings_name}/parameters/{env_name}_12312354.yml"
    with open(settings_path, 'r') as f:
        settings = yaml.safe_load(f)

    iris_options = IrisOptions()

    # num_trials = 1
    # for k in settings.keys():
    #     if k!='num_trials':
    #         setattr(iris_options, k, settings[k])
    #     else:
    #         num_trials = settings[k]

    if "num_trials" in settings.keys():
        num_trials = settings["num_trials"]

    options = IrisNp2Options()
    options.sampling_strategy = IrisNp2SamplingStrategy.kGreedySampler
    options.sampled_iris_options.epsilon = settings["admissible_proportion_in_collision"]
    options.sampled_iris_options.delta = settings["delta"]
    options.sampled_iris_options.tau = settings["tau"]
    options.ray_sampler_options.ray_search_num_steps = settings["face_ray_steps"]
    options.sampled_iris_options.max_iterations = settings["iteration_limit"]
    options.ray_sampler_options.num_particles_to_walk_towards = settings["particle_batch_size"]
    options.sampled_iris_options.mixing_steps = settings["mixing_steps"]
    options.sampled_iris_options.require_sample_point_is_contained = settings["require_sample_point_is_contained"]
    options.sampled_iris_options.relative_termination_threshold = settings["relative_termination_threshold"]
    options.sampled_iris_options.termination_threshold = settings["termination_threshold"]
    options.sampled_iris_options.max_iterations_separating_planes = settings["max_iterations_separating_planes"]


    settings_hash = hashlib.sha1(
                        json.dumps(settings, 
                            sort_keys=True)
                            .encode('utf-8')).hexdigest()[:10]
    rob_names = get_robot_instance_names(env_name)
    robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
    checker = SceneGraphCollisionChecker(model = diagram.Clone(), 
                    robot_model_instances = robot_instances,
                    #configuration_distance_function = _configuration_distance,
                    edge_step_size = 0.125)
    
    context = plant.GetMyMutableContextFromRoot(diagram_context)
    
    domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                 plant.GetPositionUpperLimits())
    def iris_handle(pt, random_seed = iris_options.random_seed):
        plant.SetPositions(context, pt)
        options.sampled_iris_options.random_seed = random_seed
        return IrisNp2(checker, Hyperellipsoid.MakeHypersphere(1e-2, pt), domain, options)

    return iris_handle, iris_options.configuration_space_margin, settings_hash, iris_options.random_seed, num_trials