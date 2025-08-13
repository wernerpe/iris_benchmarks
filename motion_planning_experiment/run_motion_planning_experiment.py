experiment_name = "precise_parameters_fewer_zo_particles"

import contextlib
import io
import multiprocessing as mp
import os.path
import time
from collections import OrderedDict, namedtuple
from copy import copy
from functools import partial
from typing import Dict

import sys
sys.path.insert(0,'/home/rebecca/Documents/code/fork_drake/drake-build/install/lib/python3.10/site-packages')

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), '..')))
from iris_environments.environments import env_names, get_environment_builder, get_robot_instance_names

import numpy as np
import pydot
from IPython.display import SVG, display
from pydrake.all import (
    Context,
    IrisFromCliqueCoverOptions,
    IrisInConfigurationSpaceFromCliqueCover,
    Meshcat,
    MeshcatPoseSliders,
    RandomGenerator,
    RobotDiagramBuilder,
    SceneGraph,
    SceneGraphCollisionChecker,
    RandomGenerator,
    IrisNp2Options, IrisNp2SamplingStrategy, IrisNp2,
    IrisZo,
    IrisZoOptions,
    GurobiSolver,
    MosekSolver
)
from pydrake.common.value import AbstractValue
from pydrake.geometry import (
    Meshcat,
    MeshcatVisualizer,
    QueryObject,
    Rgba,
    Role,
    SceneGraph,
    Sphere,
    StartMeshcat,
)
from pydrake.geometry.optimization import (
    AffineBall,
    GraphOfConvexSetsOptions,
    HPolyhedron,
    VPolytope,
    Hyperellipsoid,
    IrisInConfigurationSpace,
    IrisOptions,
    LoadIrisRegionsYamlFile,
    Point,
    SaveIrisRegionsYamlFile,
)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.meshcat import JointSliders
from pydrake.multibody.parsing import PackageMap, Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.tree import Body
from pydrake.planning import (
    GcsTrajectoryOptimization,
    MaxCliqueSolverViaGreedy,
    VisibilityGraph,
)
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer

import pickle
import yaml

root = os.path.dirname(os.path.abspath(__file__)) 
experiment_path = root + f"/logs/{experiment_name}"
parameters_path = experiment_path + "/parameters.yml"

with open(parameters_path, 'r') as f:
    parameters = yaml.safe_load(f)

greedy_parameters_path = experiment_path + "/greedy_parameters.yml"

with open(greedy_parameters_path, 'r') as f:
    greedy_parameters = yaml.safe_load(f)

parameters_path_zo = experiment_path + "/zo_parameters.yml"

with open(parameters_path_zo, 'r') as f:
    zo_parameters = yaml.safe_load(f)

ray_parameters_path = experiment_path + "/ray_parameters.yml"

with open(ray_parameters_path, 'r') as f:
    ray_parameters = yaml.safe_load(f)

ray_sampler_parameters_path = experiment_path + "/ray_sampler_parameters.yml"


env_name = '3DOFFLIPPER'
plant_builder = get_environment_builder(env_name)
plant, scene_graph, diagram, diagram_context, plant_context, _ = plant_builder(usemeshcat=False)

with open(root + '/ellipsoids.pkl', 'rb') as f:
    ellipsoid_data = pickle.load(f)

# Reconstruct the Hyperellipsoid objects
ellipsoids = []
for data in ellipsoid_data:
    A = data['A']
    center = data['center']
    ellipsoid = Hyperellipsoid(A, center)
    ellipsoids.append(ellipsoid)

source_q = ellipsoids[parameters["vertex_sequence"][0]].center()
target_q = ellipsoids[parameters["vertex_sequence"][-1]].center()


ray_options = IrisNp2Options()
ray_options.sampling_strategy = IrisNp2SamplingStrategy.kRaySampler
greedy_options = IrisNp2Options()
greedy_options.sampling_strategy = IrisNp2SamplingStrategy.kGreedySampler
zo_options = IrisZoOptions()

for k in parameters.keys():
    if k!='num_trials_iris' and k!='num_trials_optimizer'  and k!='num_knots' and k!='solver' and k!='vertex_sequence':
        setattr(ray_options.sampled_iris_options, k, parameters[k])
        setattr(greedy_options.sampled_iris_options, k, parameters[k])
        setattr(zo_options.sampled_iris_options, k, parameters[k])

for k in greedy_parameters.keys():
    setattr(greedy_options.sampled_iris_options, k, greedy_parameters[k])

for k in zo_parameters.keys():
    if hasattr(zo_options, k):
        setattr(zo_options, k, zo_parameters[k])
    else:
        setattr(zo_options.sampled_iris_options, k, zo_parameters[k])
print(zo_options)

for k in ray_parameters.keys():
    if hasattr(ray_options.sampled_iris_options, k):
        setattr(ray_options.sampled_iris_options, k, ray_parameters[k])
    else:
        setattr(ray_options.ray_sampler_options, k, ray_parameters[k])


def solve_simple_restriction(regions, source_q, target_q):
    dim = regions[0].ambient_dimension()
    prog = MathematicalProgram()
    cost = 0
    num_knots = parameters["num_knots"]
    points_all = []
    for i, region in enumerate(regions):
        points_knot = []
        if i == 0:
            x0 = source_q
        else:
            x0 = xf
            region.AddPointInSetConstraints(prog, x0)
        points_knot.append(x0)

        if num_knots > 2:
            x_i = prog.NewContinuousVariables(num_knots - 2, dim)
            for x in x_i:
                region.AddPointInSetConstraints(prog, x)
            points_knot.extend(x_i)
            points_all.extend(x_i)
        
        if i == len(regions) - 1:
            xf = target_q
        else:
            xf = prog.NewContinuousVariables(dim, "xf")
            region.AddPointInSetConstraints(prog, xf)
            points_all.append(xf)
        points_knot.append(xf)

        points_knot = np.array(points_knot)
        differences = points_knot[1:] - points_knot[:-1]
        prog.AddQuadraticCost(np.sum(differences**2))  # TDOO change to shortest path
    
    if parameters["solver"] == "gurobi":
        solver = GurobiSolver()
    else:
        assert parameters["solver"] == "mosek"
        solver = MosekSolver()
    
    solve_times = []
    for _ in range(parameters["num_trials_optimizer"]):
        t0 = time.time()
        result = solver.Solve(prog)
        solve_time = time.time() - t0
        assert(result.is_success())
        solve_times.append(solve_time)

    solution = np.vstack((source_q, result.GetSolution(points_all), target_q))
    return solve_times,  result.get_optimal_cost(), solution

rob_names = get_robot_instance_names(env_name)
robot_instances = [plant.GetModelInstanceByName(n) for n in rob_names]
checker = SceneGraphCollisionChecker(model = diagram.Clone(), 
                robot_model_instances = robot_instances,
                edge_step_size = 0.125)
domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(),
                                 plant.GetPositionUpperLimits())

# ray_iris_times = []
# greedy_iris_times = []
# zo_iris_times = []

# ray_optimizer_times = []
# greedy_optimizer_times = []
# zo_optimizer_times = []

# ray_num_faces = []
# greedy_num_faces = []
# zo_num_faces = []

# ray_path_costs = []
# greedy_path_costs = []
# zo_path_costs = []

algs = ["ray", "greedy", "zo"]
options = [ray_options, greedy_options, zo_options]

iris_times = np.zeros((len(algs), parameters["num_trials_iris"]))
optimizer_times = np.zeros((len(algs), parameters["num_trials_iris"]))
num_faces = np.zeros((len(algs), parameters["num_trials_iris"]))
path_costs = np.zeros((len(algs), parameters["num_trials_iris"]))

for i_trial in range(parameters["num_trials_iris"]):

    for i_alg in range(3):
        options[i_alg].sampled_iris_options.random_seed = i_trial

        regions = []
        t0 = time.time()
        n_faces = []
        for i, e in enumerate(ellipsoids):
            if algs[i_alg] == "zo":
                 regions.append(IrisZo(checker, e, domain, options[i_alg]))
            else:
                regions.append(IrisNp2(checker, e, domain, options[i_alg]))
            n_faces.append(len(regions[-1].b()))

            
        iris_times[i_alg, i_trial] = time.time() - t0
        num_faces[i_alg, i_trial] = np.mean(np.array(n_faces))

        regions_to_solve_over = [regions[i] for i in parameters["vertex_sequence"]]
        print("solving convex restrictions")
        solve_times_i, cost, solution_i = solve_simple_restriction(regions_to_solve_over, source_q, target_q)

        optimizer_times[i_alg, i_trial] = np.mean(solve_times_i)
        path_costs[i_alg, i_trial] = cost

results = {}
results["algs"] = algs
results["iris_times"] = iris_times
results["optimizer_times"] = optimizer_times
results["num_faces"] = num_faces
results["path_costs"] = path_costs

with open(experiment_path + "/results.pkl", "wb") as f:
    pickle.dump(results, f)