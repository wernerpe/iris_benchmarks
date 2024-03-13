from pydrake.all import IrisInConfigurationSpace, IrisOptions
from iris_environments.environments import env_names, get_environment_builder
from benchmarks.helpers import run_custom_experiment, get_experiment_name
import os
import pickle 
from functools import partial
import importlib
import sys
root = os.path.dirname(os.path.abspath(__file__)) 
experiment_name = "sampled_iris"
experiment_path = root+f"/logs/{experiment_name}"
settings_name = 'config_1'

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

