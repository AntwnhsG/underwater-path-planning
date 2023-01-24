# Underwater-Path-Planning-AUV Path planning python scripts for AUVs using AirSim simulation

- The A* and MSP algorithms used in the scripts are from "python-pathfinding" python library of brean. Link: https://github.com/brean/python-pathfinding

- The PSO algorithm comes from annafabris. Link: https://github.com/annafabris/PSO-shortest-path

- The scripts use the csv file "eval_points_whole" to obtain data about the start and goal points

- Main scripts are astar_knownMap.py, msp_knownMap.py, PSO_known_map.py

- In the psoAlgo.py file the line 184 is commented out. If you uncomment the line it allows you to run an improved version of the pso algorithm

- To run the scripts, first unzip the Underwater_blocks_airsim file and then run blocks.exe. After that you can run the algorithms.