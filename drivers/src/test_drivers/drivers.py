import time
import pathlib

import osqp
import numpy as np
import cvxpy as cp
from scipy import sparse
from math import sin, cos, atan
from splinify.splinify import SplineTrack
from ..utils import MPC_utils


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

    SPEED_SCALER = 0.6
    CORNERS_SPEED = SPEED_SCALER*4.0
    MILD_CORNERS_SPEED = SPEED_SCALER*6.
    STRAIGHTS_SPEED = SPEED_SCALER*8
    ULTRASTRAIGHTS_SPEED *= SPEED_SCALER
    CRAZY_SPEED *= SPEED_SCALER

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
        self.track = SplineTrack('/home/gnone/edoardo-ghignone/drivers/maps/SOCHI_waypoints.txt')
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

        cur_theta = MPC_utils.find_theta([x, y], self.track, self.theta, eps = 0.1)
        self.theta = cur_theta
        idx = int((slice.stop+slice.start)/2)
        scan = space_range[idx]
        angle = (540 + 135 + idx) * self.radians_per_elem
        x_car = scan*cos(angle)
        y_car = scan*sin(angle)

        x_adv = x + cos(heading) * x_car - sin(heading) * y_car
        y_adv = y + sin(heading) * x_car + cos(heading) * y_car


        i_theta = MPC_utils.find_theta([x_adv, y_adv], self.track, cur_theta + scan, eps = 0.1)

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
        self.theta = MPC_utils.find_theta([ego_odom['pose_x'], ego_odom['pose_y']], self.track, self.theta, eps = 0.1)

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

            print(self.prev_speed, self.prev_steer)
            return self.prev_speed, self.prev_steer
            
    def midline_tracking(self, ego_odom):
        
        x = ego_odom['pose_x']
        y = ego_odom['pose_y']
        self.theta = MPC_utils.find_theta([x, y], self.track, self.theta, 0.1)
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

        cur_theta = MPC_utils.find_theta([x, y], self.track, self.theta, eps = 0.1)
        self.theta = cur_theta
        idx = int((slice.stop+slice.start)/2)
        scan = space_range[idx]
        angle = (540 + 135 + idx) * self.radians_per_elem
        x_car = scan*cos(angle)
        y_car = scan*sin(angle)

        x_adv = x + cos(heading) * x_car - sin(heading) * y_car
        y_adv = y + sin(heading) * x_car + cos(heading) * y_car


        i_theta = MPC_utils.find_theta([x_adv, y_adv], self.track, cur_theta + scan, eps = 0.1)

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
        self.theta = MPC_utils.find_theta([x, y], self.track, self.theta, 0.1)
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
        self.theta = MPC_utils.find_theta([ego_odom['pose_x'], ego_odom['pose_y']], self.track, self.theta, eps = 0.1)

        
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
