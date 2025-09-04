import hashlib
import json
import yaml
import pydrake.all as pd
from iris_environments.environments import get_robot_instance_names
import os

def get_iris_handle(env_name,
                    experiment_name,
                    settings_name, 
                    plant, 
                    diagram, 
                    diagram_context):
    
    root = os.path.dirname(os.path.abspath(__file__)) 
    common_settings_path = root + f"/{experiment_name}/{settings_name}/parameters/common_options.yml"
    with open(common_settings_path, 'r') as f:
        common_settings = yaml.safe_load(f)

    common_opts = pd.CommonSampledIrisOptions()
    for k in common_settings.keys():
        if k !="alg_opts":
            setattr(common_opts, k, common_settings[k])
    
    iris_opts = pd.IrisNp2Options()
    iris_opts.sampled_iris_options = common_opts
    iris_opts.sampling_strategy = "greedy"

    rob_names = get_robot_instance_names(env_name)
    robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
    checker = pd.SceneGraphCollisionChecker(model = diagram.Clone(), 
                    robot_model_instances = robot_instances,
                    #configuration_distance_function = _configuration_distance,
                    edge_step_size = 0.125)
    print(iris_opts)   
    domain = pd.HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                 plant.GetPositionUpperLimits())
    def iris_handle(pt, random_seed = iris_opts.sampled_iris_options.random_seed):
        iris_opts.sampled_iris_options.random_seed = random_seed
        return pd.IrisNp2(checker, pd.Hyperellipsoid.MakeHypersphere(1e-2, pt), domain, iris_opts)

    return iris_handle, iris_opts.sampled_iris_options.configuration_space_margin