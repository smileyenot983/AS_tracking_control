# Load Waypoints
#############################################



import numpy as np
import csv
import matplotlib.pyplot as plt

'''This is an implementation of pure pursuit controller(p control for
speed control and lateral geometric control for steering) for Autonomous systems homework.
Pure pursuit works well when there is no slipping of wheels as is the case.
'''

'''The way how this controller works:


'''

import math
import numpy as np
import matplotlib.pyplot as plt

class Agent:
    def __init__(self, x_init, y_init, angle_init):
        self.x_pos = x_init
        self.y_pos = y_init
        self.angle = angle_init

        #time step
        self.dt = 0.01

        #velocity of vehicle, controlled by p controller
        self.speed = 0

        # PURE PURSUIT PARAMETERS
        # wheel base for vehicle
        self.wheel_base = 2.0
        # look ahead distance
        self.look_ahead = 2.0
        # look forward gain
        self.forward_gain = 0.5

        # position of vehicle's back(because we control vehicle's back)
        self.x_rear = self.x_pos - ((self.wheel_base / 2) * math.cos(self.angle))
        self.y_rear = self.y_pos - ((self.wheel_base / 2) * math.sin(self.angle))

    #updating parameters of vehicle with new parameters from controllers
    #inputs are speed and steering angle
    def update_state(self,u,delta):
        self.x_pos += self.speed * math.cos(self.angle) * self.dt
        self.y_pos += self.speed * math.sin(self.angle) * self.dt

        #TODO check why angle is updated like this
        self.angle += self.speed / self.wheel_base * math.tan(delta) * self.dt

        self.speed += u * self.dt
        self.x_rear = self.x_pos - ((self.wheel_base / 2) * math.cos(self.angle))
        self.y_rear = self.y_pos - ((self.wheel_base / 2) * math.sin(self.angle))


    # calculate distance between vehicle's rear axle and desired point
    def calculate_distance(self, point_x, point_y):
        dist_x = self.x_rear - point_x
        dist_y = self.y_rear - point_y
        distance = math.hypot(dist_x, dist_y)
        return distance

    #here we control velocity of vehicle using proportional control
    def p_control(self, target, current):
        Kp = 2.0
        u = Kp * (target - current)
        return u

    # finding index of closest point to vehicle in our path
    def search_target_index(self, points_x, points_y):
        old_nearest_point_index = None

        # to begin the process we first have to find closest point
        if old_nearest_point_index == None:

            #calculating distance between vehicle and each point in path
            distances_x = [self.x_rear - point_x for point_x in points_x]
            distances_y = [self.y_rear - point_y for point_y in points_y]

            # finding distances from current position to all other positions in path
            distances = np.hypot(distances_x, distances_y)

            #taking the point that is closest to vehicle
            nearest_point_index = np.argmin(distances)

            old_nearest_point_index = nearest_point_index

        else:
            nearest_point_index = old_nearest_point_index

            #calculating distance to next nearest point
            distance_nearest = self.calculate_distance(points_x[nearest_point_index], points_y[nearest_point_index])

            # finding farthest point that our lookforward can see
            while True:
                distance_next_index = self.calculate_distance(points_x[nearest_point_index + 1],
                                                              points_y[nearest_point_index + 1])

                if distance_nearest < distance_next_index:
                    break

                # check if we still have points in our path
                if nearest_point_index + 1 < len(points_x):
                    nearest_point_index += 1

                distance_nearest = distance_next_index
            nearest_point_index = nearest_point_index

        # look ahead distance update
        Lf = self.forward_gain * self.speed + self.look_ahead

        # finding farthest point inside LookForward distance
        # print(points_x[nearest_point_index])
        # print(points_y[nearest_point_index])
        while Lf > self.calculate_distance(points_x[nearest_point_index], points_y[nearest_point_index]):
            # check if this is last point
            if nearest_point_index + 1 >= len(points_x):
                break
            nearest_point_index += 1

        return nearest_point_index, Lf

    #calculating needed staeering for control

    def pp_steer_control(self, points_x,points_y, pind):
        ind, Lf = self.search_target_index(points_x,points_y)

        if pind >= ind:
            ind = pind

        if ind < len(points_x):
            tx = points_x[ind]
            ty = points_y[ind]
        else:
            tx = points_x[-1]
            ty = points_y[-1]
            ind = len(points_x) - 1


        alpha = math.atan2(ty - self.y_rear, tx - self.x_rear) - self.angle

        delta = math.atan2(2.0 * self.wheel_base * math.sin(alpha),Lf)

        return delta, ind

'''Here comes tracking part'''



# Opens the waypoint file and stores it to "waypoints"
waypoints_file = 'racetrack_waypoints.txt'
waypoints_np   = None
with open(waypoints_file) as waypoints_file_handle:
        waypoints = list(csv.reader(waypoints_file_handle, 
                                delimiter=',',
                                quoting=csv.QUOTE_NONNUMERIC))
        waypoints_np = np.array(waypoints)

        """ 
	Because the waypoints are discrete and our controller performs better
        with a continuous path, here we will send a subset of the waypoints
        within some lookahead distance from the closest point to the vehicle.
        Interpolating between each waypoint will provide a finer resolution
        path and make it more "continuous". A simple linear interpolation
        is used as a preliminary method to address this issue, though it is
        better addressed with better interpolation methods (spline 
        interpolation, for example). 
        More appropriate interpolation methods will not be used here for the
        sake of demonstration on what effects discrete paths can have on
        the controller. It is made much more obvious with linear
        interpolation, because in a way part of the path will be continuous
        while the discontinuous parts (which happens at the waypoints) will 
        show just what sort of effects these points have on the controller """
        
        
# Linear interpolation computations
# Compute a list of distances between waypoints
wp_distance = []   # distance array
for i in range(1, waypoints_np.shape[0]):
        wp_distance.append(
                np.sqrt((waypoints_np[i, 0] - waypoints_np[i-1, 0])**2 +
                        (waypoints_np[i, 1] - waypoints_np[i-1, 1])**2))
                #it's easier to write it as


wp_distance.append(0)  # last distance is 0 because it is the distance
                               # from the last waypoint to the last waypoint

# Linearly interpolate between waypoints and store in a list
wp_interp      = []    # interpolated values 
                               # (rows = waypoints, columns = [x, y, v])
wp_interp_hash = []    # hash table which indexes waypoints_np
                               # to the index of the waypoint in wp_interp
interp_counter = 0     # counter for current interpolated point index

INTERP_DISTANCE_RES = 0.5
for i in range(waypoints_np.shape[0] - 1):
            # Add original waypoint to interpolated waypoints list (and append
            # it to the hash table)
        wp_interp.append(list(waypoints_np[i]))
        wp_interp_hash.append(interp_counter)   
        interp_counter+=1
            
            # Interpolate to the next waypoint. First compute the number of
            # points to interpolate based on the desired resolution and
            # incrementally add interpolated points until the next waypoint
            # is about to be reached.

        num_pts_to_interp = int(np.floor(wp_distance[i] /float(INTERP_DISTANCE_RES)) - 1)
        # print(num_pts_to_interp)
        wp_vector = waypoints_np[i+1] - waypoints_np[i]
        wp_uvector = wp_vector / np.linalg.norm(wp_vector)
        for j in range(num_pts_to_interp):
                next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                wp_interp.append(list(waypoints_np[i] + next_wp_vector))
                interp_counter+=1
        # add last waypoint at the end
wp_interp.append(list(waypoints_np[-1]))
wp_interp_hash.append(interp_counter)   
interp_counter+=1

#making matrix of interpolated points
wp_interp_np = np.array(wp_interp)
# print(wp_interp_np[:10,0])

desired_xs = list(wp_interp_np[:,0])
desired_ys = list(wp_interp_np[:,1])

vehicle = Agent(desired_xs[0],desired_ys[0],0)

# target_velocity = np.mean(wp_interp_np[:,2])
target_velocity = wp_interp_np[:,2]

target_ind, _ = vehicle.search_target_index(desired_xs,desired_ys)
lastIndex = len(desired_xs)-1
print(str(lastIndex) + 'points in path')

states_x = []
states_y = []

while lastIndex > target_ind:
    # calc control input

    # velocity control
    ui = vehicle.p_control(target_velocity[target_ind], vehicle.speed)

    # steering control
    di, target_ind = vehicle.pp_steer_control(desired_xs, desired_ys, target_ind)

    vehicle.update_state(ui, di)

    states_x.append(vehicle.x_pos)
    states_y.append(vehicle.y_pos)


plt.plot(wp_interp_np[:,0], wp_interp_np[:,1],color='r',linewidth=3.0, linestyle='dashed',label='planned path')
plt.plot(states_x, states_y,color='g',linewidth=2.0, label='obtained path')
plt.legend()
plt.show()
