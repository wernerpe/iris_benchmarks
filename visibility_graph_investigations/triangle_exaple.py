from pydrake.all import (SceneGraphCollisionChecker, 
                         Rgba, 
                         Sphere, 
                         RigidTransform, 
                         RotationMatrix,
                         VisibilityGraph)
from iris_environments.environments import plant_builder_2dof_triangle_with_hole
import ipywidgets as widgets
from functools import partial
from cspace_utils.plotting import plot_points
from cspace_utils.graphs import (greedy_clique_cover, greedy_geometric_clique_cover)
from cspace_utils.environment_helpers import get_sample_cfree_handle
import numpy as np
from cspace_utils.plotting import (plot_points, 
                                   plot_vpoly_2d_meshcat, 
                                   plot_edges_clique,
                                   plot_visibility_graph)
from cspace_utils.colors import generate_maximally_different_colors
plant, scene_graph, diagram, diagram_context, plant_context,meshcat = plant_builder_2dof_triangle_with_hole(True)
# meshcat.SetProperty('/drake/proximity','visible', True)
scene_graph_context = scene_graph.GetMyMutableContextFromRoot(
    diagram_context)
robot_instances = ['cspace_robot']
checker = SceneGraphCollisionChecker(model = diagram, 
                                     robot_model_instances = [plant.GetModelInstanceByName(r) for r in robot_instances], 
                                     edge_step_size = 0.1)

meshcat.Set2dRenderMode(RigidTransform(RotationMatrix.MakeYRotation(-np.pi/2), np.array([0,0,0.5])), xmin=-7, xmax=5, ymin=-5, ymax=7)
meshcat.SetProperty('/Lights/PointLightNegativeX', 'visible', False)
meshcat.SetProperty('/Lights/PointLightPositiveX', 'visible', False)
meshcat.SetProperty('/Lights/AmbientLight/<object>', 'intensity', 3.0)
# meshcat.SetProperty('/drake/proximity', 'visible', True)

from cspace_utils.graphs.greedy_max_geom_clique import greedy_max_geometric_clique
N = 200

def col_free(q):
    return not checker.CheckConfigCollisionFree(q)

cfree_sampler = get_sample_cfree_handle(plant.GetPositionLowerLimits(), 
                                plant.GetPositionUpperLimits(),
                                col_free)
for i in range(20):
    meshcat.Delete(f'/drake/cl_{i}')
samps,_ = cfree_sampler(N, 1000,[])
adj = VisibilityGraph(checker, samps.T, parallelize = True)

# clique = greedy_max_geometric_clique(adj, samps)
vanilla_clique_cover = greedy_clique_cover(adj, 
                                   min_clique_size=1,
                                   approach = 'mip')
clique_cover = greedy_geometric_clique_cover(adj, 
                                             samps.T, 
                                             min_gain_per_clique = 1,
                                             approach = 'greedy')

from pydrake.all import VPolytope
for i in range(20):
    meshcat.Delete(f'/drake/cl_{i}')
nodes_for_plot = np.concatenate((samps, np.zeros((samps.shape[0],1))), axis = 1) 

colors = generate_maximally_different_colors(7)
colors =[Rgba(c[0], c[1],c[2],1) for c in colors]
plot_size_edge = 0.05
for idx, clique in enumerate(clique_cover):
    col = colors[idx]
    plot_points(meshcat, 
                samps[clique], 
                f'geom/cl_{idx}',           
                color = col, 
                size = plot_size_edge*0.95)
    #plot_edges_clique(meshcat, clique, nodes_for_plot, f'cl_{idx}', col)
    if len(clique)>2:
        cliquevpoly = VPolytope(samps[clique].T).GetMinimalRepresentation()
        plot_vpoly_2d_meshcat(meshcat, 
                              cliquevpoly, 
                              f'geom/cl_{idx}', 
                              col, 
                              size =plot_size_edge,
                              translation = np.array([0,0,0.1*idx]))
    elif len(clique)==2:
        plot_edges_clique(meshcat, 
                          clique, 
                          nodes_for_plot, 
                          f'geom/cl_{idx}', 
                          col, 
                          size = plot_size_edge)
    else:
        plot_points(meshcat, 
                    samps[clique], 
                    f'cl_{idx}', 
                    color = col, 
                    size = plot_size_edge)
        
for idx, clique in enumerate(vanilla_clique_cover):
    col = colors[idx]
    plot_points(meshcat, 
                samps[clique], 
                f'vanilla/cl_{idx}',           
                color = col, 
                size = plot_size_edge*0.95)
    #plot_edges_clique(meshcat, clique, nodes_for_plot, f'cl_{idx}', col)
    if len(clique)>2:
        cliquevpoly = VPolytope(samps[clique].T).GetMinimalRepresentation()
        plot_vpoly_2d_meshcat(meshcat, 
                              cliquevpoly, 
                              f'vanilla/cl_{idx}', 
                              col, 
                              size =plot_size_edge,
                              translation = np.array([0,0,0.1*idx]))
    elif len(clique)==2:
        plot_edges_clique(meshcat, 
                          clique, 
                          nodes_for_plot, 
                          f'vanilla/cl_{idx}', 
                          col, 
                          size = plot_size_edge)
    else:
        plot_points(meshcat, 
                    samps[clique], 
                    f'vanilla/cl_{idx}', 
                    color = col, 
                    size = plot_size_edge)
print('')
# plot_size_edge = 0.05
# from pydrake.all import VPolytope

# if len(clique)>2:
#     cliquevpoly = VPolytope(samps[clique].T).GetMinimalRepresentation()
#     plot_vpoly_2d_meshcat(meshcat, 
#                             cliquevpoly, 
#                             f'cl_{1}', 
#                             colors[0], 
#                             size =plot_size_edge,
#                             translation = np.array([0,0,0.1*1]))
    

# print('')