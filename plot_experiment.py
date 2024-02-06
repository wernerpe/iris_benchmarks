import matplotlib.pyplot as plt
import os
import pickle
from iris_environments.environments import env_names
import numpy as np

def get_env_name(path):
    for e in env_names:
        if e in path:
            return e

data = {}
for e in env_names:
    data[e] = {'default': {}}
keys_stats = ['times', 'volumes', 'fraction_in_collision', 'num_faces']

root = os.path.dirname(os.path.abspath(__file__))
default_exp_path = os.listdir(root + f"/benchmarks/default_experiments")
for exp in default_exp_path:
    env_name = get_env_name(exp)
    with open(root + f"/benchmarks/default_experiments/"+exp, 'rb') as f:
        result = pickle.load(f)
        
        data[env_name]['default']['mean_stats'] = [ np.mean(result[k]) for k in keys_stats]
        data[env_name]['default']['min_stats'] = [ np.min(result[k]) for k in keys_stats]
        data[env_name]['default']['max_stats'] = [ np.max(result[k]) for k in keys_stats]

print(data)

