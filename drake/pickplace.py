from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MeshcatVisualizer,
    Simulator,
    StartMeshcat,
)


def main():
    meshcat = StartMeshcat()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    plant.Finalize()

    # meshcat
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # parser = Parser(plant, scene_graph)
    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator_context = simulator.get_mutable_context()

    # step the simulator in 0.1s intervals
    step = 0.1
    max_runtime_minute = 10.0
    max_runtime_sec = max_runtime_minute * 60

    while simulator_context.get_time() < max_runtime_sec:
        next_time = min(
            simulator_context.get_time() + step,
            max_runtime_sec,
        )
        simulator.AdvanceTo(next_time)


if __name__ == "__main__":
    main()
