import airsim
import time
import sys
import argparse
import numpy
import csv
import os.path

from os import path

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# Makes the drone fly and get Lidar data
class LidarTest:


    def __init__(self):

        # Connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.simRunConsoleCommand("t.MaxFPS 60")
        self.client.enableApiControl(True)

        self.start_grid_x = 0
        self.start_grid_y = 0 
        self.start_z = -2

        self.target_grid_x = 0
        self.target_grid_y = 0

        self.start_x = 0
        self.start_y = 0

        self.target_x = 0
        self.target_y = 0 

        self.target_a = 0
        self.target_b = 0

        self.target_k = 0
        self.target_l = 0

        self.meters_traveled = 0
        self.total_runtime = 0
        self.reached_target = "Yes"

        self.step = 5.8

        self.center_grid_y = 10
        self.start_time = 0
        self.comp_end_time = 0

        # Wind Vector
        self.wind = airsim.Vector3r(-0.1, 0.5, 0)
        self.client.simSetWind(self.wind)
        

        # Initialize the array containing the coordinates of all the obstacles
        self.matrix = numpy.ones((21,21), dtype=int)
        
    def run_a_star(self, start_grid_x, start_grid_y, end_grid_x, end_grid_y, rerun = 0):
        if(rerun == 0):
            # For one obs!!! 
            # for i in range(5, 12):
            #     for j in range(6 , 14):
            #         self.matrix[i][j] = 0

            # For many obs!!!
            for i in range(2,5):
                for j in range(9, 13):
                    self.matrix[i][j] = 0
            for i in range(14, 17):
                for j in range(11, 14):
                    self.matrix[i][j] = 0
            for i in range(8, 10):
                for j in range(6, 9):
                    self.matrix[i][j] = 0
            for i in range(4, 6):
                for j in range(3, 5):
                    self.matrix[i][j] = 0
            for i in range(5, 7):
                for j in range(17, 19):
                    self.matrix[i][j] = 0
            for i in range(8, 10):
                for j in range(12, 14):
                    self.matrix[i][j] = 0
            for i in range(13, 15):
                for j in range(5, 7):
                    self.matrix[i][j] = 0


        
        grid = Grid(matrix = self.matrix)
        start_point = grid.node(start_grid_y, start_grid_x)
        end_point = grid.node(end_grid_y, end_grid_x)

        # Pass search space over shape argument, to optimize results
        finder = AStarFinder(diagonal_movement = DiagonalMovement.only_when_no_obstacle)

        path, runs = finder.find_path(start_point, end_point, grid)
        grid.cleanup()
        return path       
     
    def set_movement(self):

        current_grid_x = self.start_grid_x
        current_grid_y = self.start_grid_y
  
        drone_x = self.start_x
        drone_y = self.start_y
        drone_z = self.start_z
        path_x = [drone_x]
        path_y = [drone_y]

        # Append the starting position to the path

        start_time = time.time()
        path = self.run_a_star(current_grid_x, current_grid_y, self.target_grid_x, self.target_grid_y)

        if(self.target_a != "" and self.target_b != ""):
            path = numpy.concatenate((path, self.run_a_star(self.target_grid_x, self.target_grid_y, self.target_grid_a, self.target_grid_b, rerun = 1)), axis = 0)
        path = numpy.concatenate((path, self.run_a_star(self.target_grid_a, self.target_grid_b, self.target_grid_k, self.target_grid_l, rerun = 1)), axis = 0)

        end_time = time.time()
        self.comp_end_time = end_time - start_time
        

        for i in range(len(path)):
            print("\n")
            print(current_grid_x, current_grid_y)
            print("\n")
    
            next_grid_x = path[i][1]
            next_grid_y = path[i][0]

            # Set current grid position according to the path 
            # Set current actual drone position according to the path
            # Add step(5.8) to the total meters traveled by the drone
            if(current_grid_x < next_grid_x):
                current_grid_x += 1
                drone_x += self.step
                self.meters_traveled += self.step  

            elif(current_grid_x > next_grid_x):
                current_grid_x -= 1
                drone_x -= self.step
                self.meters_traveled += self.step                                                    

            if(current_grid_y < next_grid_y):
                current_grid_y += 1
                drone_y += self.step
                self.meters_traveled += self.step 
                    
            elif(current_grid_y > next_grid_y):
                current_grid_y -= 1
                drone_y -= self.step
                self.meters_traveled += self.step                             

            self.client.moveToPositionAsync(drone_x, drone_y, drone_z, 3, timeout_sec=15).join()       

            # Check for collisions
            drone_state = self.client.getMultirotorState()

            collision = self.client.simGetCollisionInfo("Drone1")
            if(collision.has_collided):
                self.reached_target = "No"
                break

            self.client.hoverAsync()

            # Append the current position to the path
            position = drone_state.kinematics_estimated.position                    
            drone_x = position.x_val
            drone_y = position.y_val
            drone_z = position.z_val  
            path_x.append(round(drone_x, 2))
            path_y.append(round(drone_y, 2))     

        return None

    # Set parameters for the program to run.
    # Initializes the algorithms
    def move_on_path(self):
        self.start_time = time.time()
        self.set_movement()
        self.client.hoverAsync()
        end_time = time.time()
        self.total_runtime = end_time - self.start_time

    # Executes the method responsible for moving the drone around   
    def execute(self):
        print("Arming the drone...")
        with open('eval_points_whole.csv') as file_obj:
            heading = next(file_obj)
            reader_obj = csv.reader(file_obj)
            for row in reader_obj:
                self.start_x = int(row[0])
                self.start_y = int(row[1])
                self.target_x = int(row[3])
                self.target_y = int(row[4])
                self.target_a = row[6]
                self.target_b = row[7]
                self.target_k = row[8]
                self.target_l = row[9]

                # Establish the correct positions on the grid depending on start/ target actual positions
                self.target_grid_x = int((self.target_x / self.step) + abs((self.start_x / self.step)))
                if(self.target_a != ""):
                    self.target_a = int(self.target_a)
                    self.target_grid_a = int((self.target_a / self.step) + abs((self.start_x / self.step)))

                if(self.target_b != ""):
                    self.target_b = int(self.target_b)
                    self.target_grid_b = int(self.center_grid_y + (self.target_b / self.step)-2)     

                if(self.target_k != ""):
                    self.target_k = int(self.target_k)
                    self.target_grid_k = int((self.target_k / self.step) + abs((self.start_x / self.step)))

                if(self.target_l != ""):
                    self.target_l = int(self.target_l)
                    self.target_grid_l = int(self.center_grid_y + (self.target_l / self.step)-2)             

                if(self.start_y != 0):
                    self.start_grid_y = int(self.center_grid_y + (self.start_y / self.step)-2)
                    self.target_grid_y = int(self.center_grid_y + (self.target_y / self.step)-2)                

                else:
                    self.start_grid_y = self.center_grid_y

                self.client.reset()
                self.client.enableApiControl(True)
                self.client.armDisarm(True)

                # Set teleport position info
                start_vector = airsim.Vector3r(self.start_x, self.start_y, 0)
                teleport_pose = airsim.Pose(start_vector)      

                # Teleport drone to requested position
                self.client.simSetVehiclePose(pose=teleport_pose, ignore_collision=True)

                self.client.moveToZAsync(self.start_z, 1).join()

                # Reinitialize variables required for each episode (flight)
                self.meters_traveled = 0
                self.total_steps = 0

                self.move_on_path()
                self.client.hoverAsync()
                self.meters_traveled = round(self.meters_traveled, 3)
                self.total_runtime = round(self.total_runtime, 3)
                self.comp_end_time = round(self.comp_end_time, 3)

                data = [self.meters_traveled, self.comp_end_time, self.total_runtime, self.reached_target]
                self.log_to_file(data)

                self.reached_target = "Yes"
                self.matrix = numpy.ones((21,21), dtype=int)

    def log_to_file(self, data, path="Astar_Metrics_knownMap.csv"):
        if(not os.path.exists("Astar_Metrics_knownMap.csv")):
            header = ['Total Meters', 'Computation Time', 'Total Runtime', 'Reached Target']
            with open("Astar_Metrics_knownMap.csv", 'w') as f:
                f.write(f'{header}\n')
        with open("Astar_Metrics_knownMap.csv", "a") as f:
            f.write(f'{data}\n')                    

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()
        self.client.enableApiControl(False)
        print("Done!\n")

# Main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()