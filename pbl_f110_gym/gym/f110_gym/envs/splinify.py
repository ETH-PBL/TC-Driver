from typing import List
import numpy as np
from numpy import arctan2
from scipy.interpolate import InterpolatedUnivariateSpline as Spline

class SplineTrack():
    """
    a class to represent the splinification of the track 
    """

    def __init__(self, path_to_file, safety_margin: float= 0.0) -> None:
        """
        Args: 
            path_to_file: path to the file where the coordinates of the track are saved
            safety_margin: the track will be shrinked by this amount on both sides so that 
                the car will be at a more safe distance from the borders
        
        """
        coords = read_coords_from_file(path_to_file)
        coords = np.array(coords)
        params, self.track_length = find_params(coords[1])

        # TODO shrink the track for robustness
        for i in range(len(params)-1):
            # shrink int line
            direction = coords[1,i,:] - coords[0,i,:]
            coords[0,i,:] = coords[0,i,:] + safety_margin*direction/np.linalg.norm(direction) 

            # shrink out line 
            direction = coords[1,i,:] - coords[2,i,:]
            coords[2,i,:] = coords[2,i,:] + safety_margin*direction/np.linalg.norm(direction) 

        self.midline_x = Spline(params, np.concatenate((coords[1, :, 0], np.array([coords[1, 0, 0]]))), k = 2)
        self.midline_y = Spline(params, np.concatenate((coords[1, :, 1], np.array([coords[1, 0, 1]]))), k = 2)
        self.intline_x = Spline(params, np.concatenate((coords[0, :, 0], np.array([coords[0, 0, 0]]))), k = 2)
        self.intline_y = Spline(params, np.concatenate((coords[0, :, 1], np.array([coords[0, 0, 1]]))), k = 2)
        self.outline_x = Spline(params, np.concatenate((coords[2, :, 0], np.array([coords[2, 0, 0]]))), k = 2)
        self.outline_y = Spline(params, np.concatenate((coords[2, :, 1], np.array([coords[2, 0, 1]]))), k = 2)

    def get_angle(self, theta, line = 'mid') -> float:	
        """
        Returns the angle wrt x axis tangent to the midline interpolation given the parameter theta
        """
        theta = theta%self.track_length

        if line == 'mid':
            delt_y = self.midline_y(theta, 1)  
            delt_x = self.midline_x(theta, 1)  
        elif line == 'int':
            delt_y = self.intline_y(theta, 1)  
            delt_x = self.intline_x(theta, 1)
        elif line == 'out':
            delt_y = self.outline_y(theta, 1)  
            delt_x = self.outline_x(theta, 1)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        angle = arctan2(delt_y, delt_x)
        if angle<0:
            angle += 2*np.pi

        return angle

    def get_coordinate(self, theta, line = 'mid'):
        """
        Returns the coordinate of the point corresponding to theta on the chosen line. 
        
        Args: 
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid', 'out'. Default is 'mid'.

        Returns:
            coord: 2-d coordinate
        """
        theta = theta%self.track_length

        if line == 'mid':
            coord = [self.midline_x(theta), self.midline_y(theta)]
        elif line == 'int':
            coord = [self.intline_x(theta), self.intline_y(theta)]
        elif line == 'out':
            coord = [self.outline_x(theta), self.outline_y(theta)]
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return coord

    def get_derivative(self, theta, line = 'mid'):
        """
        Returns the derivative of the point corresponding to theta on the chosen line. 
        
        Args: 
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid', 'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            der = [self.midline_x(theta, 1), self.midline_y(theta, 1)]
        elif line == 'int':
            der = [self.intline_x(theta, 1), self.intline_y(theta, 1)]
        elif line == 'out':
            der = [self.outline_x(theta, 1), self.outline_y(theta, 1)]
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return der

    def get_curvature(self, theta, line='mid'):
        """
        Returns the derivative of the point corresponding to theta on the chosen line. 
        
        Args: 
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid', 'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            x_d = self.midline_x(theta, 1)
            x_dd = self.midline_x(theta, 2)
            y_d = self.midline_y(theta, 1)
            y_dd = self.midline_y(theta, 2)
        elif line == 'int':
            x_d = self.intline_x(theta, 1)
            x_dd = self.intline_x(theta, 2)
            y_d = self.intline_y(theta, 1)
            y_dd = self.intline_y(theta, 2)
        elif line == 'out':
            x_d = self.outline_x(theta, 1)
            x_dd = self.outline_x(theta, 2)
            y_d = self.outline_y(theta, 1)
            y_dd = self.outline_y(theta, 2)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")
        
        curvature = np.abs(x_d * y_dd - y_d * x_dd)/pow((x_d**2 + y_d **2), 1.5)

        return curvature

    def get_dphi_dtheta(self, theta, line='mid'):
        """
        Returns the derivative of the angle corresponding to theta on the chosen line. 
        
        Args: 
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid', 'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            x_d = self.midline_x(theta, 1)
            x_dd = self.midline_x(theta, 2)
            y_d = self.midline_y(theta, 1)
            y_dd = self.midline_y(theta, 2)
        elif line == 'int':
            x_d = self.intline_x(theta, 1)
            x_dd = self.intline_x(theta, 2)
            y_d = self.intline_y(theta, 1)
            y_dd = self.intline_y(theta, 2)
        elif line == 'out':
            x_d = self.outline_x(theta, 1)
            x_dd = self.outline_x(theta, 2)
            y_d = self.outline_y(theta, 1)
            y_dd = self.outline_y(theta, 2)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")
        
        derivative = (x_d * y_dd - y_d * x_dd)/(x_d**2 + y_d **2)

        return derivative

    def find_theta(self, coord: np.ndarray, theta_est, eps: float = 0.01):
        """
        Find the parameter of the track nearest to the point given an approximation of it.

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track 
            theta_est: best guess at theta
            eps: precision at which the algorithm looks around for theta
        """

        found = False
        max_dist = self.track_length/20

        dist = np.linalg.norm(coord - np.array(self.get_coordinate(theta_est)))
        min_dist = dist
        min_theta = theta_est
        theta_i = theta_est

        while not found:
            theta_i += eps
            dist = np.linalg.norm(coord - np.array(self.get_coordinate(theta_i)))

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i
            else:
                found = True

            if theta_i - theta_est >= max_dist:
                found = True

            #print(theta_i)
        
        # now try to look backwards
        found = False
        theta_i = theta_est
        while not found:
            theta_i -= eps

            dist = np.linalg.norm(coord - np.array(self.get_coordinate(theta_i)))

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i
            else:
                found = True

            if theta_est - theta_i >= max_dist:
                found = True
            
            #print(theta_i)

        return min_theta

    def is_coord_behind(self, coord: np.ndarray, theta: float):
        angle = self.get_angle(theta)
        norm_vec = np.array([np.cos(angle), np.sin(angle)])
        coord_on_boundary = self.get_coordinate(theta)
        coord_forward = self.get_coordinate(theta + 0.01)
        q_forward = np.dot(coord_forward, norm_vec)
        q_bound = np.dot(coord_on_boundary, norm_vec)
        q_coord = np.dot(coord, norm_vec)

        if (q_coord-q_bound)*(q_forward-q_bound) < 0:
            return True
        else: 
            return False

class Trajectory():
    """
    A class that defines only a trajectory, basically like a track but only one line
    """
    def __init__(self, path_to_file) -> None:
        """
        Args: 
            path_to_file: path to the file where the coordinates of the track are saved
            safety_margin: the track will be shrinked by this amount on both sides so that 
                the car will be at a more safe distance from the borders
        
        """
        coords = read_sing_coords_from_file(path_to_file)
        coords = np.array(coords)
        params, self.track_length = find_params(coords)

        k = 2
        self.line_x = Spline(params, np.concatenate((coords[:, 0], np.array([coords[0, 0]]))), k = k)
        self.line_y = Spline(params, np.concatenate((coords[:, 1], np.array([coords[0, 1]]))), k = k)

    def get_coordinate(self, theta, line = 'mid'):
        """
        Returns the coordinate of the point corresponding to theta on the chosen line. 
        
        Args: 
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid', 'out'. Default is 'mid'.

        Returns:
            coord: 2-d coordinate
        """
        theta = theta%self.track_length

        coord = [self.line_x(theta), self.line_y(theta)]

        return coord

    def find_theta(self, coord: np.ndarray, theta_est, eps: float = 0.01):
        """
        Find the parameter of the track nearest to the point given an approximation of it.

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track 
            theta_est: best guess at theta
            eps: precision at which the algorithm looks around for theta
        """

        found = False
        max_dist = self.track_length/20

        dist = np.linalg.norm(coord - np.array(self.get_coordinate(theta_est)))
        min_dist = dist
        min_theta = theta_est
        theta_i = theta_est

        while not found:
            theta_i += eps
            dist = np.linalg.norm(coord - np.array(self.get_coordinate(theta_i)))

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i
            else:
                found = True

            if theta_i - theta_est >= max_dist:
                found = True

            #print(theta_i)
        
        # now try to look backwards
        found = False
        theta_i = theta_est
        while not found:
            theta_i -= eps

            dist = np.linalg.norm(coord - np.array(self.get_coordinate(theta_i)))

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i
            else:
                found = True

            if theta_est - theta_i >= max_dist:
                found = True
            
            #print(theta_i)

        return min_theta

def read_coords_from_file(path_to_file: str):
    out_arr = []
    mid_arr = []
    int_arr = []

    with open(path_to_file, 'r') as file:
        coords = file.readlines()

    coords = coords[1:] # discard 1st line

    for el in coords:
        out, mid, inn = el.strip().split('], [')
        out_arr.append([float(el) for el in out.strip('[],').split(',')])
        mid_arr.append([float(el) for el in mid.strip('[],').split(',')])
        int_arr.append([float(el) for el in inn.strip('[],').split(',')])
    
    return out_arr, mid_arr, int_arr

def read_sing_coords_from_file(path_to_file: str):
    arr = []

    with open(path_to_file, 'r') as file:
        coords = file.readlines()

    coords = coords[1:] # discard 1st line

    for el in coords:
        out = el.strip().split(',')
        arr.append([float(el) for el in out])
        
    return arr

def find_params(coords: np.ndarray) -> List[float]:
    """
    Finds the parameters (basically x axis for interpolation)
    Should be only used with central line.

    Returns:
        params: an array of the parameters, it must be long len(coords) + 1
    """

    cum_param = [0]

    prev_el = coords[0]
    length = 0

    for el in coords[1:]:
        length += np.linalg.norm((el-prev_el))
        prev_el = el
        cum_param.append(length)

    length += np.linalg.norm(el - coords[0])
    cum_param.append(length)

    return cum_param, length
