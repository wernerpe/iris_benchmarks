import hashlib
import json
import yaml
from pydrake.all import (FastIris, 
                         FastIrisOptions,
                         SceneGraphCollisionChecker,
                         HPolyhedron,
                         Hyperellipsoid)
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

    iris_options = FastIrisOptions()
    num_trials = 1
    for k in settings.keys():
        if k!='num_trials':
            setattr(iris_options, k, settings[k])
        else:
            num_trials = settings[k]

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
    
    domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                 plant.GetPositionUpperLimits())
    def iris_handle(pt, random_seed = iris_options.random_seed):
        iris_options.random_seed = random_seed
        return FastIris(checker, Hyperellipsoid.MakeHypersphere(1e-2, pt), domain, iris_options)

    return iris_handle, iris_options.configuration_space_margin, settings_hash, iris_options.random_seed, num_trials
