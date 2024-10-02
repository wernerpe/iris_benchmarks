from typing import List, Dict
import numpy as np
from pydrake.all import (SceneGraphCollisionChecker, 
                         VPolytope, 
                         HPolyhedron, 
                         RandomGenerator)

def check_if_clique(c: np.ndarray, vg_adj: np.ndarray) -> bool:
    """
    Check if the given set of vertices forms a clique in the graph.
    """
    subgraph = vg_adj[c, :]
    subgraph =subgraph[:,c] 
    np.fill_diagonal(subgraph, np.ones(len(c)))
    return np.all(subgraph == 1)

def check_if_geometric_clique(c: np.ndarray, vg_verts: np.ndarray, vg_adj: np.ndarray) -> bool:
    """
    Check if the given clique is a geometric clique.
    """
    clique_vertices = vg_verts[c]
    hull = VPolytope(clique_vertices.T)
    
    for i in range(len(vg_verts)):
        if i not in c:
            if hull.PointInSet(vg_verts[i]):
                # Check if this point is connected to all clique members
                if not np.all(vg_adj[i, c] == 1):
                    return False
    return True

def complete_symmetric_matrix(matrix):
    # Get the lower triangle indices
    lower_triangle = np.tril_indices(matrix.shape[0], -1)
    
    # Copy the upper triangle to the lower triangle
    matrix[lower_triangle] = matrix.T[lower_triangle]
    
    return matrix

def evaluate_clique_cover(
        visibility_graph : Dict[str, np.ndarray],
        cliques : List[np.ndarray],
        checker : SceneGraphCollisionChecker, 
        num_samples_in_cvxh : int = 1000, 
        ):
    
    gen = RandomGenerator(1337)
    
    clique_sizes = [len(c) for c in cliques]
    
    stats = {
        'num_cliques': len(cliques),
        'max_clique_size': np.max(clique_sizes),
        'min_clique_size': np.min(clique_sizes),
        'num_non_cliques': 0,
        'num_non_geometric_cliques': 0,
        'num_samples_in_cvxh_per_clique': num_samples_in_cvxh,
        'num_cliques_enclosing_found_collisions': 0,
        'num_collisions_in_cvxh_of_clique': [],
        'clique_sizes': clique_sizes,
        'non_cliques': [],
        'non_geometric_cliques': []
    }

    vg_verts = visibility_graph['nodes'].T
    vg_adj = complete_symmetric_matrix(visibility_graph['adjacency'].toarray())

    clique_vpolys = [VPolytope(vg_verts[c]) for c in cliques]
    clique_hpolys = [HPolyhedron(vp) for vp in clique_vpolys]

    non_cliques = []
    non_geometric_cliques = []
    num_collisions_in_cvxh_of_clique = []
    num_cliques_enclosing_found_collisions = 0

    for idx, c in enumerate(cliques):
        #check if proposed cliques are indeed cliques
        is_clique = check_if_clique(c, vg_adj)
        if not is_clique:
            non_cliques.append(idx)

        #check if proposed cliques are geometric
        is_geometric = check_if_geometric_clique(c, vg_verts, vg_adj)

        if not is_geometric:
            non_geometric_cliques.append(idx)

        prev = clique_hpolys[idx].ChebyshevCenter()
        region = clique_hpolys[idx]
        samples_in_cvxh = []
        for _ in range(num_samples_in_cvxh):
            prev = region.UniformSample(gen, prev, mixing_steps=100)
            samples_in_cvxh.append(prev)

        results = checker.CheckConfigsCollisionFree(np.array(samples_in_cvxh),
                                                    parallelize=True)
        
        num_collisions = len(np.where(1-1.0*np.array(results))[0])
        if num_collisions:
            num_cliques_enclosing_found_collisions +=1
        num_collisions_in_cvxh_of_clique.append(num_collisions)

    stats['num_non_cliques'] = len(non_cliques)
    stats['num_non_geometric_cliques'] = len(non_geometric_cliques)
    stats['num_cliques_enclosing_found_collisions'] = num_cliques_enclosing_found_collisions
    return stats
    