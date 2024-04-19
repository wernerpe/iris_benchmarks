import numpy as np
from pydrake.all import (
    HPolyhedron,
    IrisInConfigurationSpace,
    IrisOptions,
    SampledIrisInConfigurationSpace,
    SampledIrisOptions
)
import hashlib
import json
import yaml
import os

root = os.path.dirname(os.path.abspath(__file__)) 

def get_iris_handle(env_name,
                    settings_name,
                    plant, 
                    diagram, 
                    diagram_context):


    configuration_space_margin = 0.01
    settings_path = root +f"/{settings_name}/parameters/{env_name}_params.yml"
    with open(settings_path, 'r') as f:
        settings = yaml.safe_load(f)
    settings_hash = hashlib.sha1(
                        json.dumps(settings, 
                            sort_keys=True)
                            .encode('utf-8')).hexdigest()[:10]

    init_options = IrisOptions()
    init_options.iteration_limit = 1
    init_options.num_collision_infeasible_samples = 2
    init_options.configuration_space_margin = configuration_space_margin

    options = SampledIrisOptions()
    options.particle_batch_size = 1000
    options.max_particle_batches = 100
    options.configuration_space_margin = configuration_space_margin
    options.verbose = True

    options.target_proportion_in_collision = 1e-3
    options.target_uncertainty = 1e-3
    options.max_alternations = 1

    for k in settings.keys():
        if hasattr(options, k):
            setattr(options, k, settings[k])

    random_seed = 43
    num_trials = 5
    
    def iris_handle(seed_point: np.ndarray, random_seed = random_seed) -> HPolyhedron:
        ctx = plant.GetMyMutableContextFromRoot(diagram_context)
        plant.SetPositions(ctx, seed_point)
        foo = SampledIrisInConfigurationSpace(plant, diagram_context, options)
        n1 = len(foo.b())
        foo = foo.ReduceInequalities(tol=0)
        n2 = len(foo.b())
        print("Reduced from %d to %d halfspaces" % (n1, n2))
        return foo

    return iris_handle, configuration_space_margin, settings_hash, random_seed, num_trials