from typing import List, Dict
import numpy as np
from pydrake.all import (SceneGraphCollisionChecker, 
                         VPolytope, 
                         HPolyhedron, 
                         RandomGenerator)
import networkx as nx

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

def generate_random_samples_in_vpolytope(polytope: VPolytope, 
                                         num_samples: int, 
                                         seed: int = 0) -> np.ndarray:
    """
    Generate a random samples within a Drake VPolytope.
    
    Args:
    polytope (VPolytope): The Drake VPolytope to sample from.
    num_samples (int): Number of samples to generate.
    seed (int): Random seed for reproducibility.
    
    Returns:
    np.ndarray: Array of shape (num_samples, dim) containing the generated samples.
    """
    # Get the vertices of the polytope
    vertices = polytope.vertices()
    
    # Determine the dimension of the polytope
    dim = vertices.shape[0]
    
    # Initialize random number generator
    np.random.seed(seed)
    
    # Generate samples
    samples = []
    for _ in range(num_samples):
        # Generate random barycentric coordinates
        barycentric_coords = np.array([np.random.rand() for _ in range(vertices.shape[1])])
        barycentric_coords /= np.sum(barycentric_coords)
        
        # Compute the sample point as a convex combination of vertices
        sample = np.dot(barycentric_coords, vertices.T)
        samples.append(sample.squeeze())
    
    return np.array(samples)

def count_connected_components(adjacency_matrix):
    # Convert the NumPy adjacency matrix to a NetworkX graph
    G = nx.from_numpy_array(adjacency_matrix)
    
    # Use NetworkX to find the number of connected components
    num_components = nx.number_connected_components(G)
    
    return num_components

def evaluate_clique_cover(
        visibility_graph : Dict[str, np.ndarray],
        is_vg_sampled_uniformly: bool,
        cliques : List[np.ndarray],
        checker : SceneGraphCollisionChecker, 
        num_samples_in_cvxh : int = 1000, 
        ):
    
    gen = RandomGenerator(1337)
    
    clique_sizes = [len(c) for c in cliques]
    vg_verts = visibility_graph['nodes'].T
    vg_adj = complete_symmetric_matrix(visibility_graph['adjacency'].toarray())

    stats = {
        'num_connected_components': count_connected_components(vg_adj),
        'is_vg_uniformly_sampled': is_vg_sampled_uniformly,  
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

    clique_vpolys = [VPolytope((vg_verts[c]).T) for c in cliques]
    clique_hpolys = [HPolyhedron(vp) for vp in clique_vpolys]

    non_cliques = []
    non_geometric_cliques = []
    num_collisions_in_cvxh_of_clique = []
    num_cliques_enclosing_found_collisions = 0
    dim = clique_hpolys[0].ambient_dimension()
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
        
        
        if clique_vpolys[idx].vertices().shape[1] < dim + 1:
            samples_in_cvxh = generate_random_samples_in_vpolytope(clique_vpolys[idx], 
                                                                   num_samples_in_cvxh)
        else:
            try:
                for _ in range(num_samples_in_cvxh):
                    prev = region.UniformSample(gen, prev, mixing_steps=100)
                    samples_in_cvxh.append(prev)
            except:
                print("""Warning, HNR sampling failed, reverting to sampling in barycentric coordinates. 
                      This is nonuniform.""")
                samples_in_cvxh = generate_random_samples_in_vpolytope(clique_vpolys[idx], 
                                                                   num_samples_in_cvxh)
        results = checker.CheckConfigsCollisionFree(np.array(samples_in_cvxh),
                                                    parallelize=True)
        
        num_collisions = len(np.where(1-1.0*np.array(results))[0])
        if num_collisions:
            num_cliques_enclosing_found_collisions +=1
        num_collisions_in_cvxh_of_clique.append(num_collisions)

    stats['num_non_cliques'] = len(non_cliques)
    stats['num_non_geometric_cliques'] = len(non_geometric_cliques)
    stats['num_cliques_enclosing_found_collisions'] = num_cliques_enclosing_found_collisions
    stats['num_collisions_in_cvxh_of_clique'] = num_collisions_in_cvxh_of_clique 
    return stats
    