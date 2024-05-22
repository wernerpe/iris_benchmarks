use_drake_fork = True

import sys
normal_drake_path = '/home/rebecca/Documents/code/drake-build/install/lib/python3.10/site-packages'
fork_drake_path = '/home/rebecca/Documents/code/fork_drake/drake-build/install/lib/python3.10/site-packages'

if use_drake_fork:
    drake_path = fork_drake_path
    not_path = normal_drake_path
else:
    drake_path = normal_drake_path
    not_path = fork_drake_path

if not_path in sys.path:
    sys.path.remove(not_path)
if not drake_path in sys.path:
    sys.path.insert(0,drake_path)
from pydrake.all import IrisInConfigurationSpace, IrisOptions, RayIris
from iris_environments.environments import env_names, get_environment_builder
from benchmarks.helpers import run_custom_experiment, get_experiment_name
import os
import pickle 
from functools import partial
import importlib
import sys
root = os.path.dirname(os.path.abspath(__file__)) 
# experiment_name = "fast_iris"
experiment_name = "ray_iris"
experiment_path = root+f"/logs/{experiment_name}"
# settings_name = 'greedy_iris_recheck'
# settings_name = 'setting_7_parallelized_16'
# settings_name = 'setting_7_parallelized_1'
settings_name = "setting_7_old_commit_recheck"

def import_function_with_spec(module_name, function_name, file_path):
    """
    Imports a specific function from a module using util.spec_from_file_location.

    Args:
        module_name: The name of the module containing the function.
        function_name: The name of the function to import.
        file_path: The absolute path to the module file.

    Returns:
        The imported function, or None if not found.
    """

    spec = importlib.util.spec_from_file_location(
        module_name, file_path, loader=None, submodule_search_locations=None
    )

    if spec is None:
        print(f"Module '{module_name}' not found at path '{file_path}'.")
        return None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    try:
        return getattr(module, function_name)
    except AttributeError:
        print(f"Function '{function_name}' not found in module '{module_name}'.")
        return None

get_iris_handle = import_function_with_spec('algorithm', 
                                            'get_iris_handle', 
                                            experiment_path + f"/algorithm.py")
# Check if the module was loaded successfully
if not get_iris_handle:
    # Access and use the function from the module
    raise ValueError("cant import algorithm.py")

for env_name in env_names:
    print(env_name)
    plant_builder = get_environment_builder(env_name)
    plant, scene_graph, diagram, diagram_context, plant_context, _ = plant_builder(usemeshcat=False)
    
    iris_handle, configuration_space_margin, settings_hash = get_iris_handle(env_name,
                                                                             settings_name,
                                                                             plant,
                                                                             diagram,
                                                                             diagram_context,
                                                                            )
    if not f"{env_name}_{settings_hash}.pkl" in os.listdir(experiment_path):

        results = run_custom_experiment(env_name, 
                                        plant,
                                        diagram,
                                        iris_handle,
                                        configuration_space_margin)
        #name = get_experiment_name(env_name, settings='customBLAH')
        #save results
        with open(experiment_path+f"/{settings_name}/{env_name}_{settings_hash}.pkl", 'wb') as f:
            pickle.dump(results, f)

