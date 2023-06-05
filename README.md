# Whole-area Exploration by Autonomous Vehicle Ensemble

The use of drone swarms in contexts such as aerial monitoring and rescue missions in hostile and disaster-stricken areas is continuously growing. Small-sized Unmanned Aerial Vehicles (UAVs) are highly suitable in these contexts due to their agility and maneuverability. However, their limited battery capacity poses significant challenges, especially during missions requiring full coverage of large areas in a short time and in extreme weather conditions.

The *Context-Aware Coverage Path Planning (CACPP)* problem aims to determine the complete coverage path for UAVs while considering several operational constraints, including variable weather conditions and support for battery swapping operations through mobile ground battery-swapping stations (BSSes).

Solving an instance of the CACPP problem requires an iterative approach leveraging two synchronized optimization models for planning UAV paths and BSS routes. The model takes into account the need to continuously re-plan the mission based on weather conditions (i.e., temperature and wind), the presence of obstacles, the residual energy levels of the drones, and the relative positions of the drones and mobile BSSes.

Finding optimal solutions for large instances of the CACPP problem is computationally impracticable. Therefore, we developed a heuristic algorithm called *WEAVE (Whole-area Exploration by Autonomous Vehicle Ensemble)* - inspired by the verb 'to weave', as the generated paths resemble a woven fabric on a loom. WEAVE can solve large instances of the problem efficiently. 

The following animations demonstrate how the WEAVE algorithm operates in different environments, illustrating the coordinated movement of UAVs and mobile battery-swapping stations during area coverage missions.

<div align="center">
  <figure>
    <img src="example2.gif" alt="WEAVE algorithm simulation in controlled environment" width="700px">
    <figcaption style="font-size: 0.85em; color: #55; margin-top: 8px; max-width: 700px; text-align: center;"><br>WEAVE algorithm simulation in a controlled environment. The colored lines represent UAV coverage paths, while the orange rectangles indicate the positions of mobile battery-swapping stations.</figcaption>
  </figure>
</div>

<br>

<div align="center">
  <figure>
    <img src="example.gif" alt="WEAVE algorithm in real-world terrain" width="700px">
    <figcaption style="font-size: 0.85em; color: #55; margin-top: 8px; max-width: 700px; text-align: center;"><br>WEAVE algorithm simulation in a real-world scenario. The colored lines show UAV coverage paths, while the yellow squares highlight the locations of mobile battery-swapping stations.</figcaption>
  </figure>
</div>
