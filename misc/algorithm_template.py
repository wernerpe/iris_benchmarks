import numpy as np
from pydrake.all import (HPolyhedron)
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
    
    def iris_handle(seed_point: np.ndarray) -> HPolyhedron:
        region = None
        #PUT YOUR CODE HERE
        return region

    return iris_handle, configuration_space_margin, settings_hash