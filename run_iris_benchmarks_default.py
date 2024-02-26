from pydrake.all import IrisInConfigurationSpace, IrisOptions
from iris_environments.environments import env_names
from benchmarks.helpers import run_default_settings, get_experiment_name
import os
import pickle 

config = 'config_4'
for env_name in [env_names[-1]]:
    print(env_name)
    name = get_experiment_name(env_name, config, settings='default')
    result, is_new, settings_hash = run_default_settings(env_name, config)
    if is_new:
        print("saving result")
        with open(name+f"_{settings_hash}.pkl", 'wb') as f:            
            pickle.dump(result, f)