# Weather-Aware Coverage Path Planning for a Swarm of UAVs using Mobile Ground Stations for Battery-Swapping

This repo contains additional material for the paper _Weather-Aware Coverage Path Planning for a Swarm of UAVs using Mobile Ground Stations for Battery-Swapping_.

We presented a new problem called Weather-Aware Coverage Path Planning (WACPP) that aims to determine the complete coverage path for UAVs while considering several operational constraints, including variable weather conditions and support for battery swapping operations through mobile ground battery-swapping stations (BSSes).

To solve the WACPP problem, we proposed an iterative approach leveraging two synchronised optimisation models for planning UAV paths and BSSes routes. As the WACPP problem is NP-hard, we also showed a heuristic procedure to solve large instances of the problem.

The name of the proposed algorithm WEAVE (WEather-Aware VEhicle routing) was inspired by the verb 'to weave', as the generated paths resemble a woven fabric on a loom.

In the animated image below, the mobile ground stations are represented by moving rectangles along the grey roads, while the paths of the UAVs are colored purple, blue, and green.

![example](example.gif)

## Additional material

- [Optimisation models](python/): Python implementation of optimisation models with Google OR-Tools
- [Heuristic procedure](java/): Java implementation of the algorithms
- [Scenarios](scenarios/): Sample of scenarios used for the experiments
- [Stats](stats/): Stats of experiments
