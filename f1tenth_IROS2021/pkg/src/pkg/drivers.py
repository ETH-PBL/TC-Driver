import time
import pathlib

import osqp
import numpy as np
import cvxpy as cp
from scipy import sparse
from math import sin, cos, atan

from f110_gym.envs.base_classes import RaceCar
from pkg.src.pkg.utils.MPC import Index, find_theta, find_theta_slow, get_MPC_matrices, get_initial_warmstart, get_inputs, update_warm_start
from pkg.src.pkg.utils.splinify import SplineTrack

cur_dir = str(pathlib.Path(__file__).parent.resolve())

class GapFollower:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 8
    CORNERS_SPEED = 4.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    # addition
    MILD_CORNERS_SPEED = 6.
    MILD_CURVE_ANGLE = np.pi / 6 # 30 degrees

    ULTRASTRAIGHTS_ANGLE = np.pi / 90 # 2 deg
    ULTRASTRAIGHTS_SPEED = 11.

    CRAZY_SPEED = 20

    SPEED_SCALER = 0.9
    CORNERS_SPEED = SPEED_SCALER*4.0
    MILD_CORNERS_SPEED = SPEED_SCALER*6.
    STRAIGHTS_SPEED = SPEED_SCALER*8

    def __init__(self, scan_lap: bool= False):
        # used when calculating the angles of the LiDAR data
        self.radians_per_elem = None
        self.scan_lap = scan_lap

        self.env_ts = 0.01
        self.steer_filter = 6/10
        self.vel_filter = 4/10

        self.theta = 0
        self.MPC_params = {
            'N' : 10, # horizon
            'n_x' : 10, # state dimensionality (for a single timestep)
            'n_u' : 2, # input dimension
            'n_z': 12, # state + input dim
            'gamma' : 1, # weight for advancement in cost
            'T_s': self.env_ts, # timesteps in seconds, env.timestep should fetch it
            'start_point': [0.8007017, -0.2753365, 4.1421595], # start point in x, y, pose
            'nu_soft': 100
        }
        self.track = SplineTrack('/home/gnone/eduardo-ghignone/f1tenth_IROS2021/pkg/src/pkg/maps/SOCHI_waypoints.txt')
        self.car_params = {'mu': 1.0489,
               'C_sf': 4.718,
               'C_sr': 5.4562,
               'lf': 0.15875,
               'lr': 0.17145,
               'h': 0.074,
               'm': 3.74,
               'I': 0.04712,
               's_min': -0.4189,
               's_max': 0.4189,
               'sv_min': -3.2,
               'sv_max': 3.2,
               'v_switch':7.319,
               'a_max': 9.51,
               'v_min':-5.0,
               'v_max': 20.0,
               'width': 0.31,
               'length': 0.58}
        self.horizon = 20
        self.next_curv = np.zeros(shape=(self.horizon,1))

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        proc_ranges = np.array(ranges[135:-135])
        # sets each value to the mean over a given window
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges, x, y, heading):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)

        chosen_slice = None
        max_len = -1

        for sl in slices:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                #if self.validate_slice(sl, free_space_ranges, x, y, heading):
                max_len = sl_len
                chosen_slice = sl

        #if chosen_slice != None:
        return chosen_slice.start, chosen_slice.stop
        #else:
        #    print("No good slices!")
        #    return chosen_slice

    def validate_slice(self, slice, space_range, x, y, heading):
        """
        Checks if the slice is in the right direction
        """

        cur_theta = find_theta([x, y], self.track, self.theta, eps = 0.1)
        self.theta = cur_theta
        idx = int((slice.stop+slice.start)/2)
        scan = space_range[idx]
        angle = (540 + 135 + idx) * self.radians_per_elem
        x_car = scan*cos(angle)
        y_car = scan*sin(angle)

        x_adv = x + cos(heading) * x_car - sin(heading) * y_car
        y_adv = y + sin(heading) * x_car + cos(heading) * y_car


        i_theta = find_theta([x_adv, y_adv], self.track, cur_theta + scan, eps = 0.1)

        if i_theta <= cur_theta:
            print("car")
            print(scan)
            print(x, y)
            print(x_adv, y_adv)

            return False
        else: 
            return True

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def process_observation(self, ranges, ego_odom):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.theta = find_theta([ego_odom['pose_x'], ego_odom['pose_y']], self.track, self.theta, eps = 0.1)

        if self.is_in_corner() and self.car_infrontof(ranges):
            return self.midline_tracking(ego_odom)
        else:
            proc_ranges = self.preprocess_lidar(ranges)
            # Find closest point to LiDAR
            closest = proc_ranges.argmin()

            # Eliminate all points inside 'bubble' (set them to zero)
            min_index = closest - self.BUBBLE_RADIUS
            max_index = closest + self.BUBBLE_RADIUS
            if min_index < 0: min_index = 0
            if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
            proc_ranges[min_index:max_index] = 0

            #proc_ranges = self.find_advancements(proc_ranges, ego_odom['pose_x'], ego_odom['pose_y'], ego_odom['pose_theta'])

            # Find max length gap
            gap_start, gap_end = self.find_max_gap(proc_ranges, ego_odom['pose_x'], ego_odom['pose_y'], ego_odom['pose_theta'])

            # Find the best point in the gap
            best = self.find_best_point(gap_start, gap_end, proc_ranges)

            # Publish Drive message
            steering_angle = self.get_angle(best, len(proc_ranges))

            if self.scan_lap:
                if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                    speed = self.CORNERS_SPEED
                else:
                    speed = self.MILD_CORNERS_SPEED
            else:
                if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                    speed = self.CORNERS_SPEED
                elif abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
                    speed = self.MILD_CORNERS_SPEED
                elif abs(steering_angle) > self.ULTRASTRAIGHTS_ANGLE:
                    speed = self.STRAIGHTS_SPEED
                else:
                    speed = self.ULTRASTRAIGHTS_SPEED
                    if proc_ranges[best] >= 22:
                        speed = self.CRAZY_SPEED
            
            #print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
            #print('Speed: {}'.format(speed))
            self.prev_speed = speed
            self.prev_steer = steering_angle
            return self.prev_speed, self.prev_steer
            
    def midline_tracking(self, ego_odom):
        
        x = ego_odom['pose_x']
        y = ego_odom['pose_y']
        self.theta = find_theta([x, y], self.track, self.theta, 0.1)
        heading = ego_odom['pose_theta']
        vel = ego_odom['linear_vel_x']
        max_curv = -1 

        for i in range(self.horizon):
            c = self.track.get_curvature(self.theta+i*self.env_ts*vel)
            self.next_curv[i] = c
            if c >= max_curv:
                max_curv = c
        
        if max_curv >= 0.5:
            speed = self.CORNERS_SPEED
        elif max_curv >= 0.1:
            speed = self.MILD_CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED

        next_point = self.track.get_coordinate(self.theta+(69-6.9)*self.env_ts*vel)

        angle = atan(abs((next_point[1]-y)/(next_point[0]-x)))
        if next_point[1] <= y:
            if next_point[0] <= x:
                angle += np.pi
            else:
                angle = 2*np.pi - angle
        else:
            if next_point[0] <= x:
                angle = np.pi - angle


        steer = angle - heading
        
        #raise ValueError()
        if steer >= np.pi: 
            steer -= 2*np.pi
        
        if steer <= -np.pi:
            steer += 2*np.pi

        print("{:.2f}".format(self.theta))
        self.prev_steer = self.steer_filter*self.prev_steer + (1-self.steer_filter)*steer
        self.prev_speed = self.vel_filter*self.prev_speed + (1-self.vel_filter)*speed

        return speed, steer 

    def is_in_corner(self):
        """
        Checks if the car is near a corner, with predetermined sets
        """
        th = self.theta % self.track.track_length
        print(th)

        if th >= 80 and th <= 100:
            return True
        elif th >= 153 and th <= 165:
            return True
        elif th >= 183 and th <= 200:
            return True
        elif th >= 223 and th <= 235:
            return True
        elif th >= 223 and th <= 262:
            return True
        elif th >= 270 and th <= 286:
            return True
        elif th >= 362 and th <= 387:
            return True
        elif th >= 392 and th <= 412:
            return True
        elif th >= 430 and th <= 453:
            return True
        else: 
            print("not in corner")
            return False

    def car_infrontof(self, scans):
        prev_el = scans[0]

        for el in scans[1:]:
            if np.abs(el-prev_el)>=5 and el != 30 and prev_el != 30:
                print("CAR!")
                return True
        return False

class Advancer:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 8
    CORNERS_SPEED = 4.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    # addition
    MILD_CORNERS_SPEED = 6.
    MILD_CURVE_ANGLE = np.pi / 6 # 30 degrees

    ULTRASTRAIGHTS_ANGLE = np.pi / 90 # 2 deg
    ULTRASTRAIGHTS_SPEED = 11.

    CRAZY_SPEED = 20

    SPEED_SCALER = 1
    CORNERS_SPEED = SPEED_SCALER*4.0
    MILD_CORNERS_SPEED = SPEED_SCALER*6.
    STRAIGHTS_SPEED = SPEED_SCALER*8

    def __init__(self, scan_lap: bool= False):
        # used when calculating the angles of the LiDAR data
        self.radians_per_elem = None
        self.scan_lap = scan_lap

        self.env_ts = 0.01
        self.steer_filter = 1
        self.vel_filter = 1

        self.theta = 0
        self.MPC_params = {
            'N' : 10, # horizon
            'n_x' : 10, # state dimensionality (for a single timestep)
            'n_u' : 2, # input dimension
            'n_z': 12, # state + input dim
            'gamma' : 1, # weight for advancement in cost
            'T_s': self.env_ts, # timesteps in seconds, env.timestep should fetch it
            'start_point': [0.8007017, -0.2753365, 4.1421595], # start point in x, y, pose
            'nu_soft': 100
        }
        self.track = SplineTrack('/home/gnone/eduardo-ghignone/f1tenth_IROS2021/pkg/src/pkg/maps/SOCHI_waypoints.txt')
        self.car_params = {'mu': 1.0489,
               'C_sf': 4.718,
               'C_sr': 5.4562,
               'lf': 0.15875,
               'lr': 0.17145,
               'h': 0.074,
               'm': 3.74,
               'I': 0.04712,
               's_min': -0.4189,
               's_max': 0.4189,
               'sv_min': -3.2,
               'sv_max': 3.2,
               'v_switch':7.319,
               'a_max': 9.51,
               'v_min':-5.0,
               'v_max': 20.0,
               'width': 0.31,
               'length': 0.58}
        self.horizon = 20
        self.next_curv = np.zeros(shape=(self.horizon,1))
        self.prev_speed = 0
        self.prev_steer = 0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        proc_ranges = np.array(ranges[135:-135])
        # sets each value to the mean over a given window
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges, x, y, heading):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)

        chosen_slice = None
        max_len = -1
        advancement = False

        for sl in slices:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                if self.validate_slice(sl, free_space_ranges, x, y, heading):
                    advancement = True
                max_len = sl_len
                chosen_slice = sl

        if not advancement:
            print("No good slices for {}!".format(self.id))

        return chosen_slice.start, chosen_slice.stop, advancement

    def validate_slice(self, slice, space_range, x, y, heading):
        """
        Checks if the slice is in the right direction
        """

        cur_theta = find_theta([x, y], self.track, self.theta, eps = 0.1)
        self.theta = cur_theta
        idx = int((slice.stop+slice.start)/2)
        scan = space_range[idx]
        angle = (540 + 135 + idx) * self.radians_per_elem
        x_car = scan*cos(angle)
        y_car = scan*sin(angle)

        x_adv = x + cos(heading) * x_car - sin(heading) * y_car
        y_adv = y + sin(heading) * x_car + cos(heading) * y_car


        i_theta = find_theta([x_adv, y_adv], self.track, cur_theta + scan, eps = 0.1)

        if i_theta <= cur_theta + 0.69*1.5:
            print("next theta: {}".format(i_theta))
            print("cur theta: {}".format(cur_theta))
            return False
        else: 
            return True

    def is_in_corner(self):
        """
        Checks if the car is near a corner, with predetermined sets
        """
        th = self.theta % self.track.track_length

        if th >= 87 and th <= 100:
            return True
        elif th >= 160 and th <= 165:
            return True
        elif th >= 190 and th <= 200:
            return True
        elif th >= 230 and th <= 235:
            return True
        elif th >= 250 and th <= 262:
            return True
        elif th >= 275 and th <= 286:
            return True
        elif th >= 369 and th <= 387:
            return True
        elif th >= 399 and th <= 412:
            return True
        elif th >= 435 and th <= 444:
            return True
        elif th >= 450 and th <= 458:
            return True
        else: 
            print("not in corner")
            return False

    def in_crazy_sector(self):
        """
        Checks if the car is near a corner, with predetermined sets
        """
        th = self.theta % self.track.track_length
        crazy = False

        if th >= 0 and th <= 69:
            crazy = True
        elif th >= 458 and th <= self.track.track_length:
            crazy = True
        elif th >= 291 and th <= 369:
            crazy = True

        if crazy: 
            print("Crazy shit is happening !!11!")

        return crazy

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle
    
    def midline_tracking(self, ego_odom):
        
        x = ego_odom['pose_x']
        y = ego_odom['pose_y']
        self.theta = find_theta([x, y], self.track, self.theta, 0.1)
        heading = ego_odom['pose_theta']
        vel = ego_odom['linear_vel_x']
        max_curv = -1 

        for i in range(self.horizon):
            c = self.track.get_curvature(self.theta+i*self.env_ts*vel)
            self.next_curv[i] = c
            if c >= max_curv:
                max_curv = c
        
        if max_curv >= 0.5:
            speed = self.CORNERS_SPEED
        elif max_curv >= 0.1:
            speed = self.MILD_CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED

        next_point = self.track.get_coordinate(self.theta+(69-6.9)*self.env_ts*vel)

        angle = atan(abs((next_point[1]-y)/(next_point[0]-x)))
        if next_point[1] <= y:
            if next_point[0] <= x:
                angle += np.pi
            else:
                angle = 2*np.pi - angle
        else:
            if next_point[0] <= x:
                angle = np.pi - angle


        steer = angle - heading
        
        #raise ValueError()
        if steer >= np.pi: 
            steer -= 2*np.pi
        
        if steer <= -np.pi:
            steer += 2*np.pi

        print("Going to center line. I'm {}".format(self.id))
        return speed, steer 

    def process_observation(self, ranges, ego_odom):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.id = ego_odom['ego_id']
        self.theta = find_theta([ego_odom['pose_x'], ego_odom['pose_y']], self.track, self.theta, eps = 0.1)

        
        proc_ranges = self.preprocess_lidar(ranges)
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        #proc_ranges = self.find_advancements(proc_ranges, ego_odom['pose_x'], ego_odom['pose_y'], ego_odom['pose_theta'])

        # Find max length gap
        gap_start, gap_end, advancement = self.find_max_gap(proc_ranges, ego_odom['pose_x'], ego_odom['pose_y'], ego_odom['pose_theta'])

        
        if self.in_crazy_sector():
            self.PREPROCESS_CONV_SIZE = 3
            self.BEST_POINT_CONV_SIZE = 60
        else:
            self.PREPROCESS_CONV_SIZE = 3
            self.BEST_POINT_CONV_SIZE = 80

        print("{:.2f}".format(self.theta))
        if not advancement:
            if not self.is_in_corner():
                self.prev_speed = 0.99*self.prev_speed
                self.prev_steer = 0.99*self.prev_steer
            else:
                self.prev_speed, self.prev_steer = self.midline_tracking(ego_odom)
            return self.prev_speed, self.prev_steer
        else:
            # Find the best point in the gap
            best = self.find_best_point(gap_start, gap_end, proc_ranges)

            # Publish Drive message
            steering_angle = self.get_angle(best, len(proc_ranges))

            if self.scan_lap:
                if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                    speed = self.CORNERS_SPEED
                else:
                    speed = self.MILD_CORNERS_SPEED
            else:
                if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                    speed = self.CORNERS_SPEED
                elif abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
                    speed = self.MILD_CORNERS_SPEED
                elif abs(steering_angle) > self.ULTRASTRAIGHTS_ANGLE:
                    speed = self.STRAIGHTS_SPEED
                    if proc_ranges[best] >= 23:
                        print("mini BOOST for {}!".format(self.id))
                        speed = 9
                else:
                    speed = self.ULTRASTRAIGHTS_SPEED
                    if proc_ranges[best] >= 25:
                        print("BOOST for {}!".format(self.id))
                        speed = self.CRAZY_SPEED
            
            #print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
            #print('Speed: {}'.format(speed))
            
            self.prev_speed = self.vel_filter*speed + (1- self.vel_filter)*self.prev_speed
            self.prev_steer = self.steer_filter*steering_angle + (1- self.vel_filter)*self.prev_steer
            return self.prev_speed, self.prev_steer

class Enemy:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 8
    CORNERS_SPEED = 4.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    # addition
    MILD_CORNERS_SPEED = 6.
    MILD_CURVE_ANGLE = np.pi / 6 # 30 degrees

    ULTRASTRAIGHTS_ANGLE = np.pi / 90 # 2 deg
    ULTRASTRAIGHTS_SPEED = 11

    def __init__(self, scan_lap: bool= False):
        # used when calculating the angles of the LiDAR data
        self.radians_per_elem = None
        self.scan_lap = scan_lap

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        proc_ranges = np.array(ranges[135:-135])
        # sets each value to the mean over a given window
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def process_lidar(self, ranges):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(ranges)
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))

        if self.scan_lap:
            if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                speed = self.CORNERS_SPEED
            else:
                speed = self.MILD_CORNERS_SPEED
        else:
            if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                speed = self.CORNERS_SPEED
            elif abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
                speed = self.MILD_CORNERS_SPEED
            elif abs(steering_angle) > self.ULTRASTRAIGHTS_ANGLE:
                speed = self.STRAIGHTS_SPEED
            else:
                speed = self.ULTRASTRAIGHTS_SPEED
        #print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        #print('Speed: {}'.format(speed))
        return speed, steering_angle

class CLFollower:
    SPEED_SCALER = 0.7
    CORNERS_SPEED = SPEED_SCALER*4.0
    MILD_CORNERS_SPEED = SPEED_SCALER*6.
    STRAIGHTS_SPEED = SPEED_SCALER*8

    def __init__(self, scan_lap: bool= False):
        # used when calculating the angles of the LiDAR data
        self.radians_per_elem = None
        self.scan_lap = scan_lap

        self.env_ts = 0.01

        self.theta = 0
        self.MPC_params = {
            'N' : 10, # horizon
            'n_x' : 10, # state dimensionality (for a single timestep)
            'n_u' : 2, # input dimension
            'n_z': 12, # state + input dim
            'gamma' : 1, # weight for advancement in cost
            'T_s': self.env_ts, # timesteps in seconds, env.timestep should fetch it
            'start_point': [0.8007017, -0.2753365, 4.1421595], # start point in x, y, pose
            'nu_soft': 100
        }
        self.track = SplineTrack('/home/gnone/eduardo-ghignone/f1tenth_IROS2021/pkg/src/pkg/maps/SOCHI_waypoints.txt')
        self.car_params = {'mu': 1.0489,
               'C_sf': 4.718,
               'C_sr': 5.4562,
               'lf': 0.15875,
               'lr': 0.17145,
               'h': 0.074,
               'm': 3.74,
               'I': 0.04712,
               's_min': -0.4189,
               's_max': 0.4189,
               'sv_min': -3.2,
               'sv_max': 3.2,
               'v_switch':7.319,
               'a_max': 9.51,
               'v_min':-5.0,
               'v_max': 20.0,
               'width': 0.31,
               'length': 0.58}
        self.horizon = 20
        self.next_curv = np.zeros(shape=(self.horizon,1))

    def process_observation(self, ranges, ego_odom):
        
        x = ego_odom['pose_x']
        y = ego_odom['pose_y']
        self.theta = find_theta([x, y], self.track, self.theta, 0.1)
        heading = ego_odom['pose_theta']
        vel = ego_odom['linear_vel_x']
        max_curv = -1 

        for i in range(self.horizon):
            c = self.track.get_curvature(self.theta+i*self.env_ts*vel)
            self.next_curv[i] = c
            if c >= max_curv:
                max_curv = c
        
        if max_curv >= 0.5:
            speed = self.CORNERS_SPEED
        elif max_curv >= 0.1:
            speed = self.MILD_CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED

        next_point = self.track.get_coordinate(self.theta+(69-40)*self.env_ts*vel)

        angle = atan(abs((next_point[1]-y)/(next_point[0]-x)))
        if next_point[1] <= y:
            if next_point[0] <= x:
                angle += np.pi
            else:
                angle = 2*np.pi - angle
        else:
            if next_point[0] <= x:
                angle = np.pi - angle


        steer = angle - heading
        
        #raise ValueError()
        if steer >= np.pi: 
            steer -= 2*np.pi
        
        if steer <= -np.pi:
            steer += 2*np.pi

        print("{:.3f}".format(self.theta))

        return speed, steer 

# drives straight ahead at a speed of 5
class SimpleDriver:

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle

# drives toward the furthest point it sees
class AnotherDriver:

    def process_lidar(self, ranges):
        # the number of LiDAR points
        NUM_RANGES = len(ranges)
        # angle between each LiDAR point
        ANGLE_BETWEEN = 2 * np.pi / NUM_RANGES
        # number of points in each quadrant
        NUM_PER_QUADRANT = NUM_RANGES // 4

        # the index of the furthest LiDAR point (ignoring the points behind the car)
        max_idx = np.argmax(ranges[NUM_PER_QUADRANT:-NUM_PER_QUADRANT]) + NUM_PER_QUADRANT
        # some math to get the steering angle to correspond to the chosen LiDAR point
        steering_angle = max_idx * ANGLE_BETWEEN - (NUM_RANGES // 2) * ANGLE_BETWEEN
        speed = 5.0

        return speed, steering_angle

class DisparityExtender:
    
    CAR_WIDTH = 0.31
    # the min difference between adjacent LiDAR points for us to call them disparate
    DIFFERENCE_THRESHOLD = 2.
    SPEED = 5. 
    # the extra safety room we plan for along walls (as a percentage of car_width/2)
    SAFETY_PERCENTAGE = 300.

    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges)/8)
        return np.array(ranges[eighth:-eighth])
    
     
    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.] # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i]-ranges[i-1]))
        return differences
    
    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        angle = 2*np.arcsin(width/(2*dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx+1+i
                if next_idx >= len(ranges): break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx-1-i
                if next_idx < 0: break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """ For each pair of points we have decided have a large difference
            between them, we choose which side to cover (the opposite to
            the closer point), call the cover function, and return the
            resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        width_to_cover = (car_width/2) * (1+extra_pct/100)
        for index in disparities:
            first_idx = index-1
            points = ranges[first_idx:first_idx+2]
            close_idx = first_idx+np.argmin(points)
            far_idx = first_idx+np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(close_dist,
                    width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx,
                cover_right, ranges)
        return ranges
            
    def get_steering_angle(self, range_index, range_len):
        """ Calculate the angle that corresponds to a given LiDAR point and
            process it into a steering angle.
            Possible improvements: smoothing of aggressive steering angles
        """
        lidar_angle = (range_index - (range_len/2)) * self.radians_per_point
        steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        return steering_angle

    def process_lidar(self, ranges):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """
        self.radians_per_point = (2*np.pi)/len(ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, proc_ranges,
                self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        steering_angle = self.get_steering_angle(proc_ranges.argmax(),
                len(proc_ranges))
        speed = self.SPEED
        return speed, steering_angle

# MPCC driver!
class MPCCDriver:

    def __init__(self) -> None:
        
        # it would be great to take those from the environment, not sure it's possible
        env_ts = 0.01
        self.counter = 0
        self.MPC_params = {
            'N' : 40, # horizon
            'n_x' : 10, # state dimensionality (for a single timestep)
            'n_u' : 2, # input dimension
            'n_z': 12, # state + input dim
            'gamma' : 1e6, # weight for advancement in cost
            'T_s': env_ts, # timesteps in seconds, env.timestep should fetch it
            'start_point': [0.8162458, 1.1614572, 4.1446321], # start point in x, y, pose
            'nu_soft': 1e4, # soft constraints penalty function
            'q_c': 1, # coefficient on the contouring error
            'q_l': 1e3, # coefficient on the lag error
            'R': 1e-3*np.diag([0, 1e-1, 1]) # cost matrix for avoiding big changes in (respectively) velocity, steering input and acceleration input 
        }
        self.car_params = {'mu': 1.0489,
               'C_sf': 4.718,
               'C_sr': 5.4562,
               'lf': 0.15875,
               'lr': 0.17145,
               'h': 0.074,
               'm': 3.74,
               'I': 0.04712,
               's_min': -0.4189,
               's_max': 0.4189,
               'sv_min': -3.2,
               'sv_max': 3.2,
               'v_switch':7.319,
               'a_max': 9.51,
               'v_min':-5.0,
               'v_max': 20.0,
               'width': 0.31,
               'length': 0.58}
        self.track = SplineTrack(cur_dir + '/maps/SOCHI_waypoints.txt', safety_margin=self.car_params['width']*2.5)
        self.track_hard = SplineTrack(cur_dir + '/maps/SOCHI_waypoints.txt', safety_margin=self.car_params['width']*1.5)
        self.warm_start = get_initial_warmstart(self.MPC_params['N'], 0.4, self.MPC_params, self.track)
        self.prog = None

    def state_estimation(self):
        """
        Function that will process the lidar data and estimate the current position of the car, plus position of other cars

        Returns: 
            cars: position of self and other cars
        """
        pass

    def process_observation(self, observation):
        #print("Warm start")
        #print(self.warm_start)

        #time_1 = time.time_ns()
        # first we take the observations
        idx = observation['ego_idx']
        X = self.warm_start[:self.MPC_params['n_x']]
        X[Index.S_X] = observation['poses_x'][idx]
        X[Index.S_Y] = observation['poses_y'][idx]
        X[Index.V] = observation['linear_vels_x'][idx]
        X[Index.PHI] = observation['poses_theta'][idx]
        X[Index.DELTA] = observation['steering'][idx]
        X[Index.PHI_DOT] = observation['ang_vels_z'][idx]
        X[Index.THETA] = find_theta(np.array(X[:2]), self.track, X[Index.THETA])
        X[Index.V_K] = observation['linear_vels_x'][idx]

        U = self.warm_start[self.MPC_params['n_x']:self.MPC_params['n_z']]
        Z_hist = self.warm_start[self.MPC_params['n_z']:].reshape(self.MPC_params['N'],-1)


        # then we linearise the matrices 
        #print(X, U, Z_hist)
        P, q, A, l, u = get_MPC_matrices(X, U, Z_hist, self.MPC_params, self.track, self.track_hard, self.car_params)
        

        # cost scaling for better numerics
        scale = 0.01
        P = scale*2*P
        q = scale*q

        #print(np.allclose(P, P.T, rtol=1e-5, atol=1e-8))
        #raise ValueError()
        #print("cur vel : {}".format(X[3]))

        # we update the solver        
        if True:
            
            if self.prog == None:
                self.prog = osqp.OSQP()
                self.prog.setup(P, q, A, l, u, warm_start=True, check_termination = 25, verbose = False, max_iter = 10000, eps_dual_inf = 1e-9, eps_rel = 1e-2)
            elif self.counter == 1:
                self.prog = osqp.OSQP()
                self.prog.setup(P, q, A, l, u, warm_start=True, check_termination = 25, verbose = False, max_iter = 10000, eps_dual_inf = 1e-9, eps_rel = 1e-2)
            else:
                self.prog.update(q=q, l=l, u=u)
                self.prog.update(Px=P.data, Ax = A.data)

            self.prog.warm_start(x=self.warm_start) # will this really work w/o dual warmstart?

            # and we solve 
            res = self.prog.solve()
            #print("solution")
            #print(res.x)

            if res.info.status != 'solved':
                print(self.warm_start)
                raise ValueError('OSQP did not solve the problem!')
            self.warm_start = update_warm_start(res, self.MPC_params, self.car_params)
            inputs = res.x[self.MPC_params['n_x']:self.MPC_params['n_z']]
        else:
            print(np.linalg.eig(P.todense())[0])
            X_prog = cp.Variable((self.MPC_params['N']+1)*self.MPC_params['n_z'])
            prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(X_prog, P) + q.T @ X_prog), 
            [A @ X_prog <= u, A @ X_prog >= l])
            prob.solve( verbose=True)
            print(prob.status)

            self.warm_start = update_warm_start(X_prog.value, self.MPC_params, self.car_params)
            inputs = X_prog.value[self.MPC_params['n_x']:self.MPC_params['n_z']]

        #st, ve = get_inputs(inputs[1], inputs[0], X[Index.DELTA], X[Index.V], self.car_params['a_max'], self.car_params['v_max'], self.car_params['v_min'])
        print("steer_vel: {:.4f}\n    acceleration: {:.4f}".format(inputs[0], inputs[1]))
        #print("steer : {:.4f}".format(st))
        
        #print("acc INPUT: {}".format(inputs[1]))

        #print("time: {:.3f}".format((- time_1 + time.time_ns())*1e-6))
        self.counter += 1
        return inputs

    def process_observation_plot(self, observation):
    	#print("Warm start")
        #print(self.warm_start)

        #time_1 = time.time_ns()
        # first we take the observations
        idx = observation['ego_idx']
        X = self.warm_start[:self.MPC_params['n_x']]
        X[Index.S_X] = observation['poses_x'][idx]
        X[Index.S_Y] = observation['poses_y'][idx]
        X[Index.V] = observation['linear_vels_x'][idx]
        X[Index.PHI] = observation['poses_theta'][idx]
        X[Index.DELTA] = observation['steering'][idx]
        X[Index.PHI_DOT] = observation['ang_vels_z'][idx]
        X[Index.THETA] = find_theta(np.array(X[:2]), self.track, X[Index.THETA])
        X[Index.V_K] = observation['linear_vels_x'][idx]

        U = self.warm_start[self.MPC_params['n_x']:self.MPC_params['n_z']]
        Z_hist = self.warm_start[self.MPC_params['n_z']:].reshape(self.MPC_params['N'],-1)


        # then we linearise the matrices 
        #print(X, U, Z_hist)
        P, q, A, l, u, F_constr, f_constr = get_MPC_matrices(X, U, Z_hist, self.MPC_params, self.track, self.track_hard, self.car_params)
        

        # cost scaling for better numerics
        scale = 0.01
        P = scale*2*P
        q = scale*q

        #print(np.allclose(P, P.T, rtol=1e-5, atol=1e-8))
        #raise ValueError()
        #print("cur vel : {}".format(X[3]))

        # we update the solver        
        if True:
            
            if self.prog == None:
                self.prog = osqp.OSQP()
                self.prog.setup(P, q, A, l, u, warm_start=True, check_termination = 25, verbose = True, max_iter = 10000, eps_dual_inf = 1e-9, eps_rel = 1e-2)
            elif self.counter == 1:
                self.prog = osqp.OSQP()
                self.prog.setup(P, q, A, l, u, warm_start=True, check_termination = 25, verbose = True, max_iter = 10000, eps_dual_inf = 1e-9, eps_rel = 1e-2)
            else:
                self.prog.update(q=q, l=l, u=u)
                self.prog.update(Px=P.data, Ax = A.data)

            self.prog.warm_start(x=self.warm_start) # will this really work w/o dual warmstart?

            # and we solve 
            res = self.prog.solve()
            #print("solution")
            #print(res.x)

            if res.info.status != 'solved':
                print(self.warm_start)
                raise ValueError('OSQP did not solve the problem!')
            self.warm_start = update_warm_start(res, self.MPC_params, self.car_params)
            inputs = res.x[self.MPC_params['n_x']:self.MPC_params['n_z']]
        else:
            print(np.linalg.eig(P.todense())[0])
            X_prog = cp.Variable((self.MPC_params['N']+1)*self.MPC_params['n_z'])
            prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(X_prog, P) + q.T @ X_prog), 
            [A @ X_prog <= u, A @ X_prog >= l])
            prob.solve( verbose=True)
            print(prob.status)

            self.warm_start = update_warm_start(X_prog.value, self.MPC_params, self.car_params)
            inputs = X_prog.value[self.MPC_params['n_x']:self.MPC_params['n_z']]

        #st, ve = get_inputs(inputs[1], inputs[0], X[Index.DELTA], X[Index.V], self.car_params['a_max'], self.car_params['v_max'], self.car_params['v_min'])
        print("steer_vel: {:.4f}\n    acceleration: {:.4f}".format(inputs[0], inputs[1]))
        #print("steer : {:.4f}".format(st))
        
        #print("acc INPUT: {}".format(inputs[1]))

        #print("time: {:.3f}".format((- time_1 + time.time_ns())*1e-6))
        self.counter += 1

        steer, speed = inputs 
        trajectory = np.array([self.warm_start[::self.MPC_params['n_z']], self.warm_start[1::self.MPC_params['n_z']]])
        return steer, speed, trajectory, F_constr, f_constr
