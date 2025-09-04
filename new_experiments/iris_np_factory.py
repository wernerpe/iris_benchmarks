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
    mut_cont = plant.GetMyMutableContextFromRoot(diagram_context)
    root = os.path.dirname(os.path.abspath(__file__)) 
    common_settings_path = root + f"/{experiment_name}/{settings_name}/parameters/common_options.yml"
    with open(common_settings_path, 'r') as f:
        common_settings = yaml.safe_load(f)

    overrides_path = root + f"/{experiment_name}/{settings_name}/parameters/per_experiment_overrides.yml"
    with open(overrides_path, 'r') as f:
        overrides = yaml.safe_load(f)

    env_overrides = overrides[env_name]

    iris_opts = pd.IrisOptions()
    iris_np_settings = common_settings['alg_opts']['iris_np']
    iris_np_overrides = env_overrides['alg_opts']['iris_np'] if 'iris_np' in env_overrides['alg_opts'].keys() else {}
    print()
    for k in iris_np_settings.keys():
        setattr(iris_opts, k, iris_np_settings[k])

    print(20*'#'+'\n'+ f'ENV OVERRIDES: {iris_np_overrides}'+20*'#'+'\n')
    for k in iris_np_overrides.keys():
        setattr(iris_opts, k, iris_np_overrides[k])

    print(iris_opts)

    def iris_handle(pt, random_seed = iris_opts.random_seed):
        iris_opts.random_seed = random_seed
        plant.SetPositions(mut_cont, pt)
        return pd.IrisInConfigurationSpace(plant, mut_cont, iris_opts)

    return iris_handle, iris_opts.configuration_space_margin