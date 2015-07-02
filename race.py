from math import cos, sin,sqrt
import numpy as np
#from plot import plot, plot_trajectory, plot_covariance_2d
'''
###Enter Coordinates [x]######
          ^
          |
###Find traveling salesman problem with greedy search [x]###
   ^                   |
   |                   v
Kalman filter##Method to localize along path and return next destination[x]
   ^                   |
   |                   v
Sensor Interface#PD controller [x]
   ^                   |
   |                   v
Robot hardware (SW on server)


'''


class UserCode:
    def __init__(self):
        Kp_xy = 2
        Kp_z = 1
        Kd_xy = 1
        Kd_z = 0

        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T

        self.state = State()

        self.delta = 0.05 #margin of error where two points are equal
        self.origin = np.array([0.,0.,0.])
        self.beacon_list = [
             [1.5 , 0.5 , 0.],
             [3.  , 0.5 , 0.],
             [4.5 , 0.5 , 0.],
             [3.5 , 2.  , 0.],
             [1.5 , 3.5 , 0.],
             [3.  , 3.5 , 0.],
             [4.5 , 3.5 , 0.],


             [4.0 , 5.5 , 0.],
             [5.5 , 5.5 , 0.],
             [7.0 , 5.5 , 0.],
             [4.0 , 7.0 , 0.],
             [4.0 , 8.5 , 0.],
             [5.5 , 8.5 , 0.],
             [7.0 , 8.5 , 0.],

             [6.5 , 11. , 0.],
             [8.0 , 11. , 0.],
             [9.5 , 11. , 0.],
             [9.5 , 12.5, 0.],
             [9.5 , 9.5 , 0.]
            ]
        self.traveling_salesman()
        self.state_desired = State(np.array(self.path[0]))
        
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
        Uses dead reckoning to predict the next state using the current position and velocity
    
        :param t - simulation time
        :param dt - time difference this last invocation
        :param linear_velocity - x and y velocity in local quadrotor coordinate frame (independet of roll and pitch)
        :param yaw_velocity - velocity around quadrotor z axis (independet of roll and pitch)

        :return tuple containing linear x and y velocity control commands in local quadrotor coordinate frame (independet of roll and pitch), and yaw velocity
        '''
        x_p = np.zeros((3, 1))
        x_p[0:2] = x[0:2] + dt * np.dot(self.rotation(x[2]), u_linear_velocity)
        x_p[2]   = x[2]   + dt * u_yaw_velocity

        self.checkPath()
        return self.compute_control_command()

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



    def compute_control_command(self):
        '''
        PD controller
        :param self.state - current location and velocity of quadrotor
        :param self.state_desired - desired location of quadrotor. Desired velocity is always 0.
        :return u - control commands:
        '''
        state = self.state
        state_desired = self.state_desired
        u = self.Kp * (state_desired.position - state.position) + self.Kd * (state_desired.velocity - state.velocity)

    def checkPath(self):
        '''
        checks if a beacon is close enough that it is considered passed. If so, the next beacon is the new destination
        :param self.x - position in world coordinates
        :param self.path - ordered list of coordinates to follow
        :return None - if the quadrotor is near a coordinate in the list, it is removed:
        '''
        x = self.state.position
        x_desired = self.state_desired.position
        distance = lambda x1,x2: sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)

        if distance(x,x_desired)<= self.delta: #if the two points are sufficiently close, they are equal
            self.path.pop(0) #removes the first coordinate and the next one is the new destination
            self.state_desired = State(np.array(self.path[0]))
    
    def traveling_salesman(self):
        '''
        :param self.origin
        :param self.beacon_list - unsorted list of coordinates that need to be passed through
        :return self.path - greedy search for a fast path through beacons:
        '''
        self.path = []
        origin_point = self.origin
        print 'Finding fast path',
        #Finds the closests point to the origin and then finds the closest point to that one and so on..
        while self.beacon_list <> []:
            print '.',
            distance = lambda x: sqrt((x[0]-origin_point[0])**2 + (x[1]-origin_point[1])**2 )
            next_point = min(self.beacon_list, key=distance)
            next_point_index = self.beacon_list.index(next_point)
            self.beacon_list.pop(next_point_index)
            self.path.append(next_point)
            origin_point = next_point
        print ''
        for beacon in self.path:
            print beacon


class State:
    def __init__(self , position = np.zeros((3,1))):
        self.position = position
        self.velocity = np.zeros((3,1))


if __name__ == '__main__':
    a = UserCode()