from pydrake.all import SceneGraphCollisionChecker, VisibilityGraph
from pydrake.all import MaxCliqueSolverViaGreedy
from functools import partial
import numpy as np

#CHANGE THESE
robot_instances = [plant.GetModelInstanceByName("iiwa"), plant.GetModelInstanceByName("wsg")]
checker = SceneGraphCollisionChecker(model = diagram, 
                    robot_model_instances = robot_instances,
                    edge_step_size = 0.125)

def vgraph(points, checker, parallelize):
    ad_mat = VisibilityGraph(checker.Clone(), np.array(points).T, parallelize = parallelize)
    N = ad_mat.shape[0]
    for i in range(N):
        ad_mat[i,i] = False
    #TODO: need to make dense for now to avoid wierd nx bugs for saving the metis file.
    return  ad_mat
vgraph_handle = partial(vgraph, checker = checker, parallelize = True) 


def compute_double_greedy_clique_partition(adj_mat, min_cliuqe_size):
    cliques = []
    done = False
    adj_curr = adj_mat.copy()
    #adj_curr = 1- adj_curr
    #np.fill_diagonal(adj_curr, 0)
    for i in range(adj_curr.shape[0]):
        adj_curr[i,i] = False
    ind_curr = np.arange(adj_curr.shape[0])
    solver = MaxCliqueSolverViaGreedy()
    while not done:
        #val, ind_max_clique_local = solve_max_independent_set_integer(adj_curr, worklimit=worklimit) #solve_max_independet_set_KAMIS(adj_curr, maxtime = 5) #
        maximal_clique_bools = solver.SolveMaxClique(adj_curr)
        ind_max_clique_local = np.where(maximal_clique_bools)[0]
        #non_max_ind_local = np.arange(len(adj_curr))
        #non_max_ind_local = np.delete(non_max_ind_local, ind_max_clique_local, None)
        index_max_clique_global = np.array([ind_curr[i] for i in ind_max_clique_local])
        cliques.append(index_max_clique_global.reshape(-1))
        adj_curr = np.delete(adj_curr, ind_max_clique_local, 0)
        adj_curr = np.delete(adj_curr, ind_max_clique_local, 1)
        ind_curr = np.delete(ind_curr, ind_max_clique_local)
        if len(adj_curr) == 0 or len(cliques[-1])<min_cliuqe_size:
            done = True
    return cliques

#### generate samples

N = 100
dim = 12
pts = np.random.rand((N, dim))
#construct visibility graph
ad_mat = vgraph_handle(pts)

#compute cliques --- set min clique size to something larger than dim (for now)
cliques = compute_double_greedy_clique_partition(ad_mat, 2* dim)

ellipsoids = []

for cl in cliques:
    if len(cl)>=2*dim:
        points_clique = pts[cl]
        ellipsoid = Hyperellipsoid.MinimumVolumeCircumscribedEllipsoid(points_clique.T)
        ellipsoids.append(ellipsoid)

#inflate cliques
io = IrisOptions()
io.iteration_limit = 1

## other iris settings
for e in ellipsoids:
    io.starting_ellipsoid = e
    #call iris 