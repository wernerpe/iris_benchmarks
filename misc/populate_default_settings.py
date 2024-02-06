from iris_environments.environments import env_names
import yaml
import os

default = {
    'require_sample_point_is_contained': True,
    'iteration_limit': 100,
    'termination_threshold': 0.02,
    'relative_termination_threshold': 0.001,
    'configuration_space_margin': 0.01,
    'num_collision_infeasible_samples': 5,
    'num_additional_constraint_infeasible_samples': 5,
    'random_seed': 1234
}
root = os.path.dirname(os.path.abspath(__file__)) 
for e in env_names:
    with open(root+f"/../benchmarks/default_options/{e}_12312354.yml", "w") as f:
        yaml.dump(default, f)