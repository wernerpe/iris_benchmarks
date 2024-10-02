from pydrake.all import (Meldis, 
                         RobotDiagramBuilder,
                         LoadModelDirectives,
                         ProcessModelDirectives,
                         AddDefaultVisualization,
                         SceneGraphCollisionChecker
                         )

from iris_environments.environments import env_names, get_robot_instance_names

def load_bins_environment():
    def plant_builder_7dof_bins(usemeshcat = False):
        if usemeshcat:
            #meshcat = StartMeshcat()
            meldis = Meldis()
            meshcat = meldis.meshcat
        builder = RobotDiagramBuilder()
        plant = builder.plant()
        scene_graph = builder.scene_graph()
        parser = builder.parser()
        #parser.package_map().Add("cvisirisexamples", missing directory)
        file_p = "/home/peter/gitcspace/iris_benchmarks/iris_environments"
        directives_file = file_p+"/directives/7dof_bins_example_urdf.yaml"#FindResourceOrThrow() 
        # directives_file = file_p+"/directives/7dof_bins_example.yaml"#FindResourceOrThrow() 
        path_repo = file_p #os.path.dirname(os.path.abspath(__file__))
        parser.package_map().Add("iris_environments", path_repo+"/assets")
        directives = LoadModelDirectives(directives_file)
        models = ProcessModelDirectives(directives, plant, parser)
        plant.Finalize()
        if usemeshcat:
            #par = MeshcatVisualizerParams()
            #par.role = Role.kIllustration
            #visualizer = MeshcatVisualizer.AddToBuilder(builder.builder(), scene_graph, meshcat, par)
            visualizer = AddDefaultVisualization(builder.builder(), meshcat)
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(diagram_context)
        diagram.ForcedPublish(diagram_context)
        return plant, scene_graph, diagram, diagram_context, plant_context, meshcat if usemeshcat else None

    plant, scene_graph, diagram, diagram_context, plant_context, meshcat = plant_builder_7dof_bins(usemeshcat=True)

    scene_graph_context = scene_graph.GetMyMutableContextFromRoot(
        diagram_context)
    robot_instances = get_robot_instance_names(env_names[6])
    checker = SceneGraphCollisionChecker(model = diagram, 
                                        robot_model_instances = [plant.GetModelInstanceByName(r) for r in robot_instances], 
                                        edge_step_size = 0.1)
    return checker