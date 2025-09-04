from iris_environments.environments import get_environment_builder
import numpy as np
import ipywidgets as widgets
from functools import partial
import pydrake.all as pd
from iris_environments.environments import env_names
currname = env_names[-1]
plant_builder = get_environment_builder(currname)
plant, scene_graph, diagram, diagram_context, plant_context, meshcat = plant_builder(usemeshcat=True)

scene_graph_context = scene_graph.GetMyMutableContextFromRoot(
    diagram_context)

opts = pd.CommonSampledIrisOptions()
opts.configuration_space_margin = 0.01
opts.tau = ...
opts.delta = ...
opts.epsilon = ...
opts.mixing_steps = 50
opts.max_iterations = 1
opts.max_iterations_separating_planes = 1000
opts.max_separating_planes_per_iteration = 5




print('')