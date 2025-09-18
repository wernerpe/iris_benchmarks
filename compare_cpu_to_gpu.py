from iris_environments.environments import get_environment_builder
import numpy as np
import ipywidgets as widgets
from functools import partial
from pydrake.all import (RigidTransform, Rgba, Sphere, RotationMatrix)
from iris_environments.environments import env_names
import time
from benchmarks.helpers import load_seed_points

import pydrake.all as pd


currname = '14DOFIIWASSB'#env_names[-1]
seed_configs = load_seed_points(currname)

plant_builder = get_environment_builder(currname)
plant, scene_graph, diagram, diagram_context, plant_context, meshcat = plant_builder(usemeshcat=True)

scene_graph_context = scene_graph.GetMyMutableContextFromRoot(
    diagram_context)

from pydrake.all import QueryObject, SceneGraphInspector
scene_graph_context = scene_graph.GetMyContextFromRoot(diagram_context)
plant_context = plant.GetMyContextFromRoot(diagram_context)
query : QueryObject = scene_graph.get_query_output_port().Eval(scene_graph_context)
inspector : SceneGraphInspector = query.inspector()
meshcat.SetProperty('/drake/proximity', 'visible', True)
import pycsdecomp as csd
from drake_csd_bridge import convert_drake_plant_to_csd_plant
csd_plant, geom_name_to_id = convert_drake_plant_to_csd_plant(plant,
                                             plant_context,
                                             inspector)
robot_idx = [plant.GetModelInstanceByName('iiwa_left'),
plant.GetModelInstanceByName('iiwa_right'),
plant.GetModelInstanceByName('wsg_right'),
plant.GetModelInstanceByName('wsg_left')]
from pydrake.all import SceneGraphCollisionChecker
checker = SceneGraphCollisionChecker(model = diagram,
                           robot_model_instances = robot_idx,
                           edge_step_size = 0.1)
geom_id_to_name = {}
for k,v in geom_name_to_id.items():
    geom_id_to_name[v] = k

groups = [[7,8,9], [21,22,23]]
# 2x N matrix where n is the number of collision pairs one colum containts two geometry indices that need to be checked for collisions
cpm = csd_plant.getCollisionPairMatrix() 
csd_insp : csd.SceneInspector = csd_plant.getSceneInspector()
group_geom_idx = {}
for gr_id, group in enumerate(groups):
    group_geom_idx[gr_id] = set()
    for link_id in group:
        col_geoms = csd_insp.link_index_to_scene_collision_geometry_indices[link_id]
        group_geom_idx[gr_id].update(col_geoms)

col_to_delete = []
for filter_set in group_geom_idx.values():
    for id in range(cpm.shape[1]):
        if cpm[0,id] in filter_set and cpm[1,id] in filter_set:
            col_to_delete.append(id)
cpm_filtered = np.delete(cpm, col_to_delete, axis=1)
csd_mplant = csd_plant.getMinimalPlant()
csd_mplant.set_collision_pairs_from_matrix(cpm_filtered)


pd_domain = pd.HPolyhedron.MakeBox(plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits())

n_samples = 10000
domain = csd.HPolyhedron()
domain.MakeBox(csd_plant.getPositionLowerLimits(), csd_plant.getPositionUpperLimits())
samples = csd.UniformSampleInHPolyhedraCuda(polyhedra=[domain], 
                                        starting_points=domain.ChebyshevCenter(), 
                                        num_samples_per_hpolyhedron=n_samples, 
                                        mixing_steps=100,
                                        seed = 1337)[0]
res_csd = csd.CheckCollisionFreeCuda(samples, csd_mplant)
res_drake = checker.CheckConfigsCollisionFree(samples.T, parallelize=True)

number_differing_results = 0
critical_configs = []
for idx, (rcsd, rdr) in enumerate(zip(res_csd, res_drake)):
    if rcsd!=rdr:
        number_differing_results +=1
        critical_configs.append(samples[:, idx])
assert len(critical_configs) == 0


gpu_opts = csd.EizoOptions()
gpu_opts.bisection_steps = 9
gpu_opts.epsilon = 0.01
gpu_opts.tau = 0.5
gpu_opts.delta = 0.05
gpu_opts.configuration_margin = 0.01
# gpu_opts.num_particles = 50000 set in the for loop
gpu_opts.mixing_steps = 50
gpu_opts.max_hyperplanes_per_iteration = 1
gpu_opts.max_iterations = 1


comm_cpu_opts = pd.CommonSampledIrisOptions()
comm_cpu_opts.delta = gpu_opts.delta
comm_cpu_opts.epsilon = gpu_opts.epsilon
comm_cpu_opts.tau = gpu_opts.tau
comm_cpu_opts.configuration_space_margin = gpu_opts.configuration_margin
comm_cpu_opts.num_particles = gpu_opts.num_particles
comm_cpu_opts.mixing_steps = gpu_opts.mixing_steps
comm_cpu_opts.max_iterations = 1
comm_cpu_opts.max_iterations_separating_planes = gpu_opts.max_iterations
comm_cpu_opts.max_separating_planes_per_iteration = gpu_opts.max_hyperplanes_per_iteration
comm_cpu_opts.remove_all_collisions_possible = False
comm_cpu_opts.sample_particles_in_parallel = True
comm_cpu_opts.verbose = True

cpu_opts = pd.IrisZoOptions()
cpu_opts.sampled_iris_options = comm_cpu_opts
cpu_opts.bisection_steps = gpu_opts.bisection_steps

def get_region_stats(r: pd.HPolyhedron, time, seed_point_idx, trial_idx):
    starting_points =r.ChebyshevCenter()
    r_csd = csd.HPolyhedron(r.A(), r.b())
    n_samp =int(1e2)
    samples = csd.UniformSampleInHPolyhedraCuda(polyhedra=[r_csd], 
                                        starting_points=starting_points, 
                                        num_samples_per_hpolyhedron=n_samp, 
                                        mixing_steps=100,
                                        seed = 1337)[0]
    res = checker.CheckConfigsCollisionFree(samples.T, parallelize = pd.Parallelism(20))
    #res = csd.CheckCollisionFreeCuda(samples, csd_mplant)
    
    stats = {
        'el_vol': r.MaximumVolumeInscribedEllipsoid().CalcVolume(),
        'num_faces': r.A().shape[0],
        'time': time,
        'frac_in_collision': np.sum(1-np.array(res))/n_samp,
        'seed_point_idx': seed_point_idx,
        'trial_idx': trial_idx,
        'num_particles': gpu_opts.num_particles,
        'region': r
    }
    return stats

for part in [50, 500, 5000, 50000]:
    all_drake_stats = []
    all_csd_stats = []
    for trial in range(10):
        for idx, sp in enumerate(seed_configs):
            print(f"part {part} trial {trial} seed idx {idx}")
            plant.SetPositions(plant_context, sp)
            diagram.ForcedPublish(diagram_context)
            print(checker.CheckConfigCollisionFree(sp))
            print(csd.CheckCollisionFree(sp, csd_mplant))
            cpu_opts.sampled_iris_options.random_seed = 1337 + trial
            cpu_opts.sampled_iris_options.num_particles = part
            t1 = time.time()
            r = pd.IrisZo(checker, 
                        pd.Hyperellipsoid.MakeHypersphere(1e-3, sp), 
                        pd_domain, 
                        cpu_opts)
            t2 = time.time()
            t_drake = t2-t1
            
            #this can be done offline so it is not  counted as comp time
            gpu_opts.seed = 1337 + trial
            gpu_opts.num_particles = part
            edge_inflator = csd.CudaEdgeInflator(csd_mplant, [0], gpu_opts, domain)
            t1 = time.time()
            r2 = edge_inflator.inflateEdge(sp.reshape(-1,1), sp.reshape(-1,1), csd.Voxels(), 0.01, True)
            t2 = time.time()
            r2_drake = pd.HPolyhedron(r2.A(), r2.b())
            t_csd = t2-t1
            drake_stats = get_region_stats(r, t_drake, idx, trial)
            csd_stats = get_region_stats(r2_drake,t_csd, idx, trial)
            
            all_drake_stats.append(drake_stats)
            all_csd_stats.append(csd_stats)

    res = {'drake': all_drake_stats, 'csd': all_csd_stats}
    import pickle
    with open(f'new_experiments/gpu_vs_cpu_comparison_{gpu_opts.max_hyperplanes_per_iteration}_{gpu_opts.max_iterations}_{gpu_opts.num_particles}_{gpu_opts.mixing_steps}.pkl', 'wb') as f:
        pickle.dump(res, f)
        