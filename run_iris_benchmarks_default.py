from iris_environments.environments import env_names
from benchmarks.helpers import run_default_settings, get_experiment_name
import os
import pickle 
import argparse

def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('env', type=int, help='an integer for the environment')
    args = parser.parse_args()
    
    config = 'config_precise'
    if args.env ==-1:
        envs = env_names[1:]
    else:
        envs = [env_names[args.env]]
    for env_name in envs:
        print(env_name)
        name = get_experiment_name(env_name, config, settings='default')
        result, is_new, settings_hash = run_default_settings(env_name, config)
        if is_new:
            print("saving result")
            with open(name+f"_{settings_hash}.pkl", 'wb') as f:            
                pickle.dump(result, f)
    
if __name__ == "__main__":
    main()