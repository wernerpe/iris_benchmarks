from pydrake.all import IrisInConfigurationSpace, IrisOptions
from iris_environments.environments import env_names
from benchmarks.helpers import run_default_settings, get_experiment_name
import os
import pickle 


results_default = {}

for env_name in env_names:
    result, is_new = run_default_settings(env_name)
    name = get_experiment_name(env_name, settings='default')
    if is_new:
        with open(name, 'wb') as f:
            pickle.dump(result, f)