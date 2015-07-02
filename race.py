from math import cos, sin,sqrt
import numpy as np
#from plot import plot, plot_trajectory, plot_covariance_2d
'''
###Enter Coordinates######
          ^
          |
###Find traveling salesman problem with greedy search [x]###
   ^                   |
   |                   v
Kalman filter##Method to localize along path and return next destination
   ^                   |
   |                   v
Sensor Interface#PD controller
   ^                   |
   |                   v
Robot hardware (SW on server)


'''


class UserCode:
    def __init__(self):
        self.origin = np.array([0.,0.,0.])
        self.beacon_list = [(1,1,1),(3,4,1),(10,10,10)]
        self.traveling_salesman()
        
    def get_markers(self):
        '''
        place up to 30 markers in the world
        '''
        markers = [
             [0, 0], # marker at world position x = 0, y = 0
             [2, 0]  # marker at world position x = 2, y = 0
        ]
        
        #TODO: Add your markers where needed
       
        return markers
        
    def state_callback(self, t, dt, linear_velocity, yaw_velocity):
        '''
        called when a new odometry measurement arrives approx. 200Hz
    
        :param t - simulation time
        :param dt - time difference this last invocation
        :param linear_velocity - x and y velocity in local quadrotor coordinate frame (independet of roll and pitch)
        :param yaw_velocity - velocity around quadrotor z axis (independet of roll and pitch)

        :return tuple containing linear x and y velocity control commands in local quadrotor coordinate frame (independet of roll and pitch), and yaw velocity
        '''
        
        return np.ones((2,1)) * 0.1, 0


    def measurement_callback(self, marker_position_world, marker_yaw_world, marker_position_relative, marker_yaw_relative):
        '''
        called when a new marker measurement arrives max 30Hz, marker measurements are only available if the quadrotor is
        sufficiently close to a marker
            
        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :param marker_position_relative - x and y position of the marker relative to the quadrotor 2x1 vector
        :param marker_yaw_relative - orientation of the marker relative to the quadrotor
        '''
        pass
    
    def traveling_salesman(self):
        '''
        :param self.origin
        :param self.beacon_list - unsorted list of coordinates that need to be passed through
        :return self.path - greedy search for a fast path through beacons:
        '''
        self.path = []
        origin_point = self.origin

        #Finds the closests point to the origin and then finds the closest point to that one and so on..
        while self.beacon_list <> []:
            distance = lambda x: sqrt((x[0]-origin_point[0])**2 + (x[1]-origin_point[1])**2 + (x[2]-origin_point[2])**2)
            next_point = min(self.beacon_list, key=distance)
            next_point_index = self.beacon_list.index(next_point)
            self.beacon_list.pop(next_point_index)
            self.path.append(next_point)
            origin_point = next_point

        for beacon in self.path:
            print beacon

if __name__ == '__main__':
    a = UserCode()