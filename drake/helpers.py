def run_sim_loop(simulator, context):
    # step the simulator in 0.1s intervals
    step = 0.1
    max_runtime_minute = 10.0
    max_runtime_sec = max_runtime_minute * 60

    while context.get_time() < max_runtime_sec:
        next_time = min(
            context.get_time() + step,
            max_runtime_sec,
        )
        simulator.AdvanceTo(next_time)
