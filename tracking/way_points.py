# Load Waypoints
#############################################



import numpy as np
import csv
import matplotlib.pyplot as plt

from pure_pursuit import *

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