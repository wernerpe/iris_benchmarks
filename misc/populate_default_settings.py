from iris_environments.environments import env_names
import yaml
import os

config = 'config_5'
default = {
    'require_sample_point_is_contained': True,
    'iteration_limit': 2,
    'termination_threshold': 0.02,
    'relative_termination_threshold': 0.001,
    'configuration_space_margin': 0.01,
    'num_collision_infeasible_samples': 10,
    'num_additional_constraint_infeasible_samples': 1,
    'random_seed': 1234
}

fio = {'num_particles': 50, 
       'target_uncertainty': 0.1, 
       'admissible_proportion_in_collision': 0.05, 
       'max_iterations': 2, 
       'max_iterations_separating_planes': 100,
       'max_separating_planes_per_iteration': -1, 
       'bisection_steps': 9, 
       'verbose': False, 
       'configuration_space_margin': 0.01, 
       'termination_threshold': 0.02, 
       'relative_termination_threshold': 0.001, 
       'random_seed': 1234}


sio = {
'particle_batch_size': 100, 
'target_uncertainty': 0.1, 
'target_proportion_in_collision': 0.05, 
'max_particle_batches': 200, 
'max_alternations': 2, 
'verbose': True, 
'require_sample_point_is_contained': True, 
'configuration_space_margin': 0.01, 
'termination_threshold': 0.02, 
'relative_termination_threshold': 0.001, 
'random_seed': 1234}

# root = os.path.dirname(os.path.abspath(__file__)) 
# for e in env_names:
#     with open(root+f"/../benchmarks/default_experiments/{config}/parameters/{e}_12312354.yml", "w") as f:
#         yaml.dump(default, f)


# root = os.path.dirname(os.path.abspath(__file__)) 
# for e in env_names:
#     with open(root+f"/../logs/fast_iris/{config}/parameters/{e}_12312354.yml", "w") as f:
#         yaml.dump(fio, f)

root = os.path.dirname(os.path.abspath(__file__)) 
assert len(os.listdir(root+f"/../logs/sampled_iris/{config}/parameters/"))==0
    
for e in env_names:
    with open(root+f"/../logs/sampled_iris/{config}/parameters/{e}_params.yml", "w+") as f:
        print(f"populating {root+f'/../logs/sampled_iris/{config}/parameters/{e}_params.yml'}")
        yaml.dump(sio, f)
print('done')