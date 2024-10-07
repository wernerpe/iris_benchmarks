from cspace_utils.graphs import (compute_greedy_clique_partition,
                                 double_greedy_clique_cover,
                                 double_greedy_clique_cover_nx)

from evaluate_clique_cover_approach import evaluate_clique_cover
from load_environment import load_bins_environment
import os 
import pickle

cwd = os.path.dirname(__file__)

approach_long_name = ['greedy_mip', 
                      'double_greedy_drake', 
                      'double_greedy_nx']
approach_short_name = ['gmip',
                       'dgdr',
                       'dgnx']
import numpy as np

def complete_symmetric_matrix(matrix):
    # Get the lower triangle indices
    lower_triangle = np.tril_indices(matrix.shape[0], -1)
    
    # Copy the upper triangle to the lower triangle
    matrix[lower_triangle] = matrix.T[lower_triangle]
    
    return matrix

def get_clique_cover(adj, 
                     vertices, 
                     approach):
    
    assert approach in approach_long_name
    adj = adj.toarray()
    adj = complete_symmetric_matrix(adj)

    if approach == 'greedy_mip':
        return compute_greedy_clique_partition(adj, 1, 200)
    elif approach == 'double_greedy_drake':
        return double_greedy_clique_cover(adj, 1)
    elif approach == 'double_greedy_nx':
        return double_greedy_clique_cover_nx(adj, 1)
    else:
        raise NotImplementedError
    
checker = load_bins_environment()

approach = 'double_greedy_drake'
short_name_approach = approach_short_name[approach_long_name.index(approach)]
dir_vgs = cwd+'/graphs/iiwa_bins/vg_7_dof_bins'
vg_names = os.listdir(dir_vgs)

for vg in vg_names[::-1]:
    res_name = cwd + '/graphs/iiwa_bins/clique_covers'
    run_string = res_name+f"/{short_name_approach}_{vg}"
    if f"{short_name_approach}_{vg}" in os.listdir(res_name):
        continue
    # if not "4000" in vg:
    #     continue

    with open(dir_vgs+f"/{vg}" , 'rb') as f:
        data = pickle.load(f)    
    vertices = data['nodes']
    adjacency = data['adjacency']
    clique_cover = get_clique_cover(adjacency, vertices, approach)
    stats = evaluate_clique_cover(data,
                                  True if 'uniform' in vg_names else False, 
                                  clique_cover, 
                                  checker, 1000)
    print(stats)
    data = {'cliques': clique_cover, 
            'adjacency': adjacency, 
            'vertices': vertices,
            'stats': stats}
    
    with open(run_string, 'wb') as f:
        pickle.dump(data, f)


