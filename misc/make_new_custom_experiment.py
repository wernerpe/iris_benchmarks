import os
from iris_environments.environments import env_names
import shutil

experiment_name = "my_experiment2"
file_loc = os.path.dirname(os.path.abspath(__file__)) 
exp_dir = file_loc + f"/../logs/{experiment_name}"
if not os.path.exists(exp_dir):
    os.mkdir(exp_dir)

if not os.path.exists(exp_dir+f"/config_1"):
    os.mkdir(exp_dir+f"/config_1")
if not os.path.exists(exp_dir+f"/config_1/parameters"):
    os.mkdir(exp_dir+f"/config_1/parameters")
shutil.copy(file_loc+"/algorithm_template.py", exp_dir+"/algorithm.py")
for e in env_names:
    with open(exp_dir+f"/config_1/parameters/{e}_params.yml", 'w+') as f:
        f.write("params: 0")

    