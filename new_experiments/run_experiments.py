from iris_environments.environments import env_names, get_environment_builder
import yaml
import pickle
from utils import import_function_with_spec
from benchmarks.helpers import run_custom_experiment

experiment_name = 'run1'
settings = 'precise'
with open(f"new_experiments/{experiment_name}/experiment_opts.yml", 'r') as f:
    experiment_opts = yaml.safe_load(f)
num_trials = 10#experiment_opts['num_trials']

algs = ['iris_zo', 'iris_np', 'iris_np2_greedy', 'iris_np2_ray']
alg = algs[1]
get_iris_handle = import_function_with_spec(f'{alg}_factory', 'get_iris_handle',
                                            f'new_experiments/{alg}_factory.py')


for env_name in env_names[1:]:
    print('#'*20+env_name+'#'*20)
    plant_builder = get_environment_builder(env_name)
    plant, scene_graph, diagram, diagram_context, plant_context, _ = plant_builder(usemeshcat=False)
    
    iris_handle, configuration_space_margin = get_iris_handle(env_name, 
                                  experiment_name, 
                                  settings, 
                                  plant, 
                                  diagram, 
                                  diagram_context)
    
    results = run_custom_experiment(env_name, 
                                    plant,
                                    diagram,
                                    iris_handle,
                                    configuration_space_margin,
                                    1234,
                                    num_trials)
    #name = get_experiment_name(env_name, settings='customBLAH')
    #save results
    with open(f"new_experiments/{experiment_name}/{settings}/{alg}/{env_name}.pkl", 'wb') as f:
        pickle.dump(results, f)