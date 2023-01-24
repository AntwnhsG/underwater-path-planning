import airsim
import time
import sys
import argparse
import numpy
import csv
import os.path

from os import path

from psoAlgo import PsoAlgo

# Makes the drone fly and get Lidar data
class LidarTest:


    def __init__(self):

        # Connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.simRunConsoleCommand("t.MaxFPS 60")
        self.client.enableApiControl(True)

        self.start_x = 0
        self.start_y = 0
        self.start_z = -2

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
        self.comp_time = 0
        self.obs_co = []

        # Wind Vector
        self.wind = airsim.Vector3r(-0.1, 0.5, 0)
        self.client.simSetWind(self.wind)

        # Initialize the array containing the coordinates of all the obstacles
        self.matrix = numpy.ones((21,21), dtype=int)
        
    def run_pso(self, start_x, start_y, end_x, end_y, rerun = 0):
        pso_path = PsoAlgo(start_point=(start_x, start_y), end_point=(end_x, end_y))
        if(rerun == 0):
            # For one obs!!! 
            # with open('obs_one.csv') as file_obj:
            #     heading = next(file_obj)
            #     reader_obj = csv.reader(file_obj)
            #     for row in reader_obj:
            #         a = row[0]
            #         b = row[1]
            #         self.obs_co.append([float(a), float(b)])

            # For many obs!!!
            with open('obs.csv') as file_obj:
                heading = next(file_obj)
                reader_obj = csv.reader(file_obj)
                for row in reader_obj:
                    a = row[0]
                    b = row[1]
                    self.obs_co.append([float(a), float(b)])

        pso_path.set_obstacles(self.obs_co)
        path = pso_path.main()
        
        # Pass search space over shape argument, to optimize results

        return path       


    def set_movement(self):
        path_list = []
        current_x = self.start_x
        current_y = self.start_y
  
        drone_x = self.start_x
        drone_y = self.start_y
        drone_z = self.start_z

        # Append the starting position to the path
        path_list.append(numpy.array([self.start_x, self.start_y, self.start_z]))

        start_time = time.time()
        path = self.run_pso(current_x, current_y, self.target_x, self.target_y)

        if(self.target_a != "" and self.target_b != ""):
            self.target_a = int(self.target_a)
            self.target_b = int(self.target_b)
            self.target_k = int(self.target_k)
            self.target_l = int(self.target_l)
            path = numpy.concatenate((path, self.run_pso(self.target_x, self.target_y, self.target_a, self.target_b, rerun = 1)), axis = 0)
            path = numpy.concatenate((path, self.run_pso(self.target_a, self.target_b, self.target_k, self.target_l, rerun = 1)), axis = 0)
        end_time = time.time()
        self.comp_time = end_time - start_time
        for i in range(len(path)):
            print("\n")
            print(current_x, current_y)
            print("\n")

            next_x = path[i][0]
            next_y = path[i][1]

            # Set current grid position according to the path 
            # Set current actual drone position according to the path
            # Add step(5.8) to the total meters traveled by the drone
            if(current_x < next_x):
                drone_x = next_x
                self.meters_traveled += abs(abs(next_x) - abs(current_x))    
                current_x = next_x   

            if(current_x > next_x):                   
                    drone_x = next_x
                    self.meters_traveled += abs(abs(current_x) - abs(next_x))       
                    current_x = next_x                                      

            if(current_y < next_y):
                    drone_y = next_y
                    self.meters_traveled += abs(abs(next_y) - abs(current_y)) 
                    current_x = next_x
                            
            elif(current_y > next_y):
                    drone_y = next_y
                    self.meters_traveled += abs(abs(current_y) - abs(next_y))   
                    current_x = next_x                          

            self.client.moveToPositionAsync(drone_x, drone_y, self.start_z, 3, timeout_sec=10).join()

            # Check for collisions
            drone_state = self.client.getMultirotorState(vehicle_name="Drone1")
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
            path_list.append(numpy.array([drone_x, drone_y, drone_z]))         


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
                self.obs_co = []

                self.move_on_path()
                self.client.hoverAsync()

                self.meters_traveled = round(self.meters_traveled, 3)
                self.total_runtime = round(self.total_runtime, 3)
                self.comp_time = round(self.comp_time, 3)
                data = [self.meters_traveled, self.comp_time, self.total_runtime, self.reached_target]
                self.log_to_file(data)

                self.reached_target = "Yes"
                self.matrix = numpy.ones((21,21), dtype=int)

    def log_to_file(self, data, path="PSO_metrics_KnownMap.csv"):
        if(not os.path.exists("PSO_metrics_KnownMap.csv")):
            header = ['Total Meters', 'Computation Time', 'Total Runtime', 'Reached Target']
            with open("PSO_metrics_KnownMap.csv", 'w') as f:
                f.write(f'{header}\n')
        with open("PSO_metrics_KnownMap.csv", "a") as f:
            f.write(f'{data}\n')                    

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()
        self.client.enableApiControl(False)
        print("Done!\n")

    def parse_lidarData(self, data):

        # Reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points


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