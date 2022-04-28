import os
import pathlib
import time

import osqp
import yaml
import numpy as np
import cvxpy as cp
from scipy import sparse
from math import sin, cos, atan
from forcespro import nlp

from .utils import index
from .utils import MPC_utils
from .utils import comp_utils
from .utils import solv_utils
from splinify.splinify import SplineTrack

Index = index.Index
NlIndex = index.NlIndex

cur_dir = str(pathlib.Path(__file__).parent.resolve())
config_dir = cur_dir + "/../config/"

with open(config_dir + "MPC_params.yaml", "r") as conf:
    MPC_params = yaml.safe_load(conf)

with open(config_dir + "car_params.yaml", "r") as conf:
    car_params = yaml.safe_load(conf)

with open(config_dir + "MPCC_debug.yaml", "r") as conf:
    DEBUG = yaml.safe_load(conf)


class MPCCDriver:
    """
    The mpcc driver

    """

    def __init__(self, dir_to_track_waypoints, slow_threshold, MPC_params = MPC_params) -> None:

        self.counter = 0
        self.MPC_params = MPC_params
        self.car_params = car_params
        if DEBUG["solver"] == "nlp_forces" or DEBUG["solver"] == "forces":
            self.track = SplineTrack(
                dir_to_track_waypoints, safety_margin=self.car_params["width"] * 2.0
            )
        else:
            self.track = SplineTrack(
                dir_to_track_waypoints, safety_margin=self.car_params["width"] * 2.0
            )
        self.track_hard = SplineTrack(
            dir_to_track_waypoints, safety_margin=self.car_params["width"] * 2.0
        )
        self.start_point = np.concatenate(
            (
                self.track.get_coordinate(MPC_params["start_par"]),
                self.track.get_angle(MPC_params["start_par"]),
            ),
            axis=None,
        )
        # self.warm_start = MPC_utils.get_serious_init_ws(self.track, self.start_point)
        self.warm_start = MPC_utils.get_initial_warmstart(
           MPC_params["N"],
           0.5,
           MPC_params,
           self.track,
           self.start_point,
           MPC_params["start_par"],
        )

        self.prog = None
        self.slow_threshold = slow_threshold
        self.prev_input = np.zeros(2)

        cur_machine = os.environ["USER"]
        if DEBUG["solver"] == "forces":
            self.forces_solver = solv_utils.SolverTaco(cur_machine, MPC_params['N'])

        if DEBUG["solver"] == "nlp_forces":
            self.nlp_solver_fast = nlp.Solver.from_directory(
                cur_dir + "/../utils/FORCESNLPsolver_fast/"
            )
            self.nlp_solver_slow = nlp.Solver.from_directory(
                cur_dir + "/../utils/FORCESNLPsolver_slow/"
            )

    def process_state(self, state):
        """
        Process the 7 dimensional state as observed, update the state and get the inputs
        """

        print_deb = False
        n_z = self.MPC_params["n_z"]
        COMPUTATIONAL_DELAY = 0 # TODO config this (computational delay)

        time1 = time.perf_counter()
        if state[Index.V] < 0.05:
            # 1.a) if the car is still we want a new warmstart
            theta = self.track.find_theta_slow(state[:2], eps=0.2)

            # forward-propagating the state to account for the computation delay
            if COMPUTATIONAL_DELAY > 0:
                state = np.array(
                    comp_utils.dynamics(
                        np.concatenate((state, [theta, 0]), axis=None),
                        self.warm_start[MPC_params['n_x']:MPC_params['n_z']],
                        COMPUTATIONAL_DELAY,
                        state[Index.V] < 0.1,
                    )
                )
            if print_deb:
                print("phi")
            if print_deb:
                print(state[Index.PHI])
            if print_deb:
                print("delta")
            if print_deb:
                print(state[Index.DELTA])

            
            self.warm_start = MPC_utils.get_initial_warmstart(
                self.MPC_params["N"],
                v_x0 = 0.5,
                a_0 = 0,
                MPC_params = self.MPC_params,
                track = self.track,
                start_point = np.concatenate(
                    (state[Index.S_X : Index.S_Y + 1], [state[Index.PHI]]), axis=None
                ),
                start_theta = theta,
            )
            if print_deb:
                print("phis")
            if print_deb:
                print(self.warm_start[Index.PHI::n_z])

        else:
            # 1.b) if not we just forward-propagate the state to account for the computation delay
            theta = self.track.find_theta(state[:2], self.warm_start[Index.THETA], eps = 0.05)
            if COMPUTATIONAL_DELAY > 0:
                state = np.array(
                    comp_utils.dynamics(
                        np.concatenate((state, [theta, 0]), axis=None),
                        self.prev_input,
                        COMPUTATIONAL_DELAY,
                        state[Index.V] < 0.1,
                    )
                )
        time2 = time.perf_counter()
        if print_deb:
            print("Time for forward prop state: {}".format(time2-time1))

        # print(self.warm_start[Index.PHI::n_z])

        # 2) preprocesses the internal state based on the observation
        time1 = time.perf_counter()
        X, U, Z_hist = self.get_from_state(state, theta)
        if print_deb:
            print("error warm start - measurement")
        if print_deb:
            print(-X+self.warm_start[:10])
        time2 = time.perf_counter()
        print("Time for update state: {}".format(time2-time1))

        
        # print(X[Index.S_X], Z_hist[:, Index.S_X])
        # print(X[Index.S_Y], Z_hist[:, Index.S_Y])
        # print(X[Index.DELTA], Z_hist[:, Index.DELTA])
        # print(X[Index.PHI], Z_hist[:, Index.PHI])
        # print(X[Index.PHI_DOT], Z_hist[:, Index.PHI_DOT])
        # print(X[Index.BETA], Z_hist[:, Index.BETA])
        # print(X[Index.V], Z_hist[:, Index.V])
        # print(X[Index.THETA], Z_hist[:, Index.THETA])
        

        # 3) we solve the problem
        time1 = time.perf_counter()
        _, _, _, _, _, accelerations, steer_vels, x_traj, y_traj = self.solve(X, U, Z_hist, solver=DEBUG["solver"])
        time2 = time.perf_counter()
        if print_deb:
            print("Time for solve: {}".format(time2-time1))

        
        # print("trajectory input (acc, stvels)")
        
        # print(accelerations, steer_vels)

        # 4) we get the inputs and retransform them to speed and steer as ROS does not accept acc/steer vel
        # time1 = time.perf_counter()
        # n_z = self.MPC_params["n_z"]
        # x_traj = self.warm_start[Index.S_X :: n_z]
        # y_traj = self.warm_start[Index.S_Y :: n_z]
        # accelerations = self.warm_start[Index.ACC :: n_z]
        # steer_vels = self.warm_start[Index.V_DELTA :: n_z]
        # steers = []
        # speeds = []
        # print(steer_vels, accelerations)
        # for i, (acc, stvel) in enumerate(zip(accelerations, steer_vels)):
        #     print(self.warm_start[n_z*i + Index.DELTA], self.warm_start[n_z*i + Index.V])          
        #     st, sp = MPC_utils.get_inputs(acc, stvel, self.warm_start[n_z*i + Index.DELTA], self.warm_start[n_z*i + Index.V])
        #     #print(st, sp)
        #     steers.append(st)
        #     speeds.append(sp)
        # print(steers, speeds)
        # time2 = time.perf_counter()
        # print("Time for getting proper inputs: {}".format(time2-time1))

        self.counter += 1

        
        time1 = time.perf_counter()
        theta = self.warm_start[Index.THETA]
        intline = np.array(
            [self.track.get_coordinate(theta + 0.5*i, line="int") for i in range(20)]
        )
        outline = np.array(
            [self.track.get_coordinate(theta + 0.5*i, line="out") for i in range(20)]
        )
        time2 = time.perf_counter()
        if print_deb:
            print("Time for getting trajectory lines: {}".format(time2-time1))
        return x_traj, y_traj, steer_vels, accelerations, outline, intline

    def process_observation(self, observation):
        # print("Warm start")
        # print(self.warm_start)

        # time_1 = time.time_ns()
        # first we take the observations
        X, U, Z_hist, _ = self.update_from_observation(observation)

        # SECOND we solve the problem
        inputs, F_constr, f_constr, max_P_el, info_solv, _, _, _, _ = self.solve(
            X, U, Z_hist, solver=DEBUG["solver"]
        )

        steer, speed = inputs
        # st, ve = get_inputs(inputs[1], inputs[0], X[Index.DELTA], X[Index.V], self.car_params['a_max'], self.car_params['v_max'], self.car_params['v_min'])
        # print(
        #     "steer_vel: {:.4f}\n    acceleration: {:.4f}".format(inputs[0], inputs[1])
        # )
        
        # print("time: {:.3f}".format((- time_1 + time.time_ns())*1e-6))
        self.counter += 1
        return steer, speed

    def process_observation_plot(self, observation):
        # print("Warm start")
        # print(self.warm_start)

        # FIRST we take the observations
        X, U, Z_hist, idx = self.update_from_observation(observation)

        # SECOND we solve the problem
        inputs, F_constr, f_constr, max_P_el, info_solv, _, _, _, _ = self.solve(
            X, U, Z_hist, solver=DEBUG["solver"]
        )

        # st, ve = get_inputs(inputs[1], inputs[0], X[Index.DELTA], X[Index.V], self.car_params['a_max'], self.car_params['v_max'], self.car_params['v_min'])
        # print("steer_vel: {:.4f}\n    acceleration: {:.4f}".format(inputs[0], inputs[1]))

        # print("steer : {:.4f}".format(st))
        # print("acc INPUT: {}".format(inputs[1]))

        self.counter += 1

        steer, speed = inputs
        # steer, speed = MPC_utils.get_inputs(
        #     speed, 
        #     steer, 
        #     observation["steering"][idx], 
        #     observation["linear_vels_x"][idx],
        #     )
        print("steer:{}  speed: {}".format(steer, speed))
        trajectory = np.array(
            [
                self.warm_start[Index.S_X :: self.MPC_params["n_z"]],
                self.warm_start[Index.S_Y :: self.MPC_params["n_z"]],
                self.warm_start[Index.THETA :: self.MPC_params["n_z"]],
            ]
        )
        cur_speed = observation["linear_vels_x"][idx]
        cur_angle = observation["poses_theta"][idx]
        cur_steer = observation["steering"][idx]
        soft_cons = self.warm_start[Index.SOFT :: MPC_params["n_z"]]
        cur_theta = X[Index.THETA]
        return (
            steer,
            speed,
            trajectory,
            cur_speed,
            cur_angle,
            cur_steer,
            cur_theta,
            F_constr,
            f_constr,
            max_P_el,
            soft_cons,
            info_solv,
        )

    def update_from_state(self, state, theta):
        """
        Preprocesses the internal state of the driver based on the observation obtained from the
        gym environment

        Args:
            state: the observed state of the car

        Returns:
            X: 9-D current state
            U: 2-D previous input, predicted by MPC at the previous step
            Z_hist: rest of the state trajectory that is used as warmstart
        """
        X = np.zeros(MPC_params['n_x'])
        X[:7] = state

        # the following part manages the angle wrap-around
        if state[Index.PHI] - self.warm_start[Index.PHI] >= np.pi:
            X[Index.PHI] = state[Index.PHI] - 2 * np.pi
        elif self.warm_start[Index.PHI] - state[Index.PHI] >= np.pi:
            X[Index.PHI] = state[Index.PHI] + 2 * np.pi
        else:
            X[Index.PHI] = state[Index.PHI]

        # we update the proj. of the current pos. of the car on the midline
        X[Index.THETA] = self.track.find_theta(
            np.array(X[:2]), theta, eps=0.1
        )

        X[Index.V_K] = self.warm_start[Index.V_K]

        # we also return the previously computed input and the rest of 
        # the previous solution as it will be used as warmstart
        U = self.warm_start[self.MPC_params["n_x"] : self.MPC_params["n_z"]]
        Z_hist = self.warm_start[self.MPC_params["n_z"] :].reshape(
            self.MPC_params["N"], -1
        )

        return X, U, Z_hist
    
    def get_from_state(self, state, theta):
        """
        Obtains warm start from async state
        """

        n_z = MPC_params['n_z']
        n_x = MPC_params['n_x']

        X = np.zeros(MPC_params['n_x'], dtype=np.float64)
        X[:7] = state

        # the following part manages the angle wrap-around
        if state[Index.PHI] - self.warm_start[Index.PHI] >= np.pi:
            X[Index.PHI] = state[Index.PHI] - 2 * np.pi
        elif self.warm_start[Index.PHI] - state[Index.PHI] >= np.pi:
            X[Index.PHI] = state[Index.PHI] + 2 * np.pi
        else:
            X[Index.PHI] = state[Index.PHI]

        # here we update the projection of the current position of the car on the midline
        X[Index.THETA] = self.track.find_theta(
            np.array(X[:2]), 
            theta, 
            eps=0.1,
        )

        X[Index.V_K] = self.warm_start[Index.V_K]
        U = self.warm_start[self.MPC_params["n_x"] : self.MPC_params["n_z"]]
        Z_hist = np.zeros((MPC_params['N'], MPC_params['n_z']), dtype=np.float64)
        
        for i in range(MPC_params['N']):
            if i == 0:
                next_st, U = MPC_utils.constr_dynamics(
                    X, U,
                    MPC_params['T_s'],
                    X[Index.V] < self.slow_threshold,
                )
                Z_hist[0, :Index.THETA] = next_st
                Z_hist[0, Index.THETA] = self.track.find_theta(
                    Z_hist[0, :Index.S_Y+1], 
                    self.warm_start[(i+1)*n_z + Index.THETA],
                    )
                Z_hist[0, Index.V_K] = self.warm_start[(i+1)*n_z + Index.V_K]
            else:
                cur_u = self.warm_start[(i*n_z)+n_x:(i+1)*n_z]
                next_st, constr_u = MPC_utils.constr_dynamics(
                    Z_hist[i-1, :Index.THETA], 
                    cur_u, 
                    MPC_params['T_s'], 
                    Z_hist[i-1, Index.V] < self.slow_threshold,
                )
                Z_hist[i-1, n_x:n_z] = constr_u
                

                Z_hist[i, :Index.THETA] = next_st
                Z_hist[i, Index.THETA] = self.track.find_theta(
                    Z_hist[i, :Index.S_Y+1], 
                    self.warm_start[(i+1)*n_z + Index.THETA],
                    )
                Z_hist[i, Index.V_K] = self.warm_start[(i+1)*n_z + Index.V_K]
                Z_hist[i, n_x:n_z] = self.warm_start[(i+1)*n_z + n_x:(i+2)*n_z]

        return X, U, Z_hist

    def update_from_observation(self, observation):
        """
        Updates the internal state of the driver based on the observation obtained from the
        gym environment

        Args:
            observation: the gym environment observation

        Returns:
            X: 10-D current state
            U: 2-D previous input, predicted by MPC at the previous step
            Z_hist: rest of the state trajectory that is used as warmstart
            idx: ego index (0 if first car, 1 if second car etc.)
        """
        idx = observation["ego_idx"]
        X = self.warm_start[: self.MPC_params["n_x"]]
        # print(X)
        X[Index.S_X] = observation["poses_x"][idx]
        X[Index.S_Y] = observation["poses_y"][idx]

        if (
            observation["linear_vels_x"][idx] != 0
            or observation["linear_vels_y"][idx] != 0
        ):
            X[Index.V] = np.linalg.norm(
                [observation["linear_vels_x"][idx], observation["linear_vels_y"][idx]]
            )
            X[Index.BETA] = np.arctan(
                observation["linear_vels_y"][idx]
                / observation["linear_vels_x"][idx]
            )
        else:
            X[Index.V] = 0
            X[Index.BETA] = 0
        
        X[Index.V] = np.clip(X[Index.V], 0, self.MPC_params['upper_bound']['v']-0.0001)

        # print("slipp: {}".format(X[Index.BETA]))

        # the following part manages the angle wrap-around
        if observation["poses_theta"][idx] - X[Index.PHI] >= np.pi:
            X[Index.PHI] = observation["poses_theta"][idx] - 2 * np.pi
        elif X[Index.PHI] - observation["poses_theta"][idx] >= np.pi:
            X[Index.PHI] = observation["poses_theta"][idx] + 2 * np.pi
        else:
            X[Index.PHI] = observation["poses_theta"][idx]
        X[Index.PHI] = np.clip(X[Index.PHI], self.MPC_params['lower_bound']['phi'], self.MPC_params['upper_bound']['phi'])

        # print("PHI: {}".format(X[Index.PHI]))
        # print("obs theta {}".format(observation['poses_theta'][idx]))

        X[Index.DELTA] = np.clip(observation["steering"][idx], self.MPC_params['lower_bound']['delta'], self.MPC_params['upper_bound']['delta'])
        X[Index.PHI_DOT] = np.clip(observation["ang_vels_z"][idx], self.MPC_params['lower_bound']['phi_dot'], self.MPC_params['upper_bound']['phi_dot'])
        # here we update the projection of the current position of the car on the midline
        X[Index.THETA] = self.track.find_theta(
            np.array(X[:2]), X[Index.THETA], eps=0.005
        )
        X[Index.BETA] = np.clip(X[Index.BETA], -np.pi/4, np.pi/4)
        # print(X[Index.THETA])
        # print(X)

        # we also return the previously computed input and the rest of the previous solution as it will be used as warmstart
        U = self.warm_start[self.MPC_params["n_x"] : self.MPC_params["n_z"]]
        Z_hist = self.warm_start[self.MPC_params["n_z"] :].reshape(
            self.MPC_params["N"], -1
        )
        return X, U, Z_hist, idx

    def solve(self, X, U, Z_hist, solver: str = "osqp"):

        if solver == "osqp":
            info_solv = {}

            time1 = time.perf_counter()
            P, q, A, l, u, F_constr, f_constr = MPC_utils.get_MPC_matrices(
                X,
                U,
                Z_hist,
                self.track,
                self.track_hard,
                self.car_params,
                slow_threshold=self.slow_threshold,
            )
            # A += np.ones(A.shape)*1e-6
            A = sparse.csc_matrix(A)
            max_P_el = np.abs(A).max()

            # cost scaling for better numerics
            scale = 0.01
            P = scale * 2 * P
            q = scale * q

            time2 = time.perf_counter()
            # print("Getting matrices time in s: {}".format(time2-time1))
            info_solv["get_matr_time"] = time2 - time1

            time1 = time.perf_counter()
            if self.prog == None:
                info_solv["full_setup"] = 1
                self.prog = osqp.OSQP()
                self.prog.setup(
                    P,
                    q,
                    A,
                    l,
                    u,
                    warm_start=True,
                    check_termination=25,
                    verbose=False,
                    polish=False,
                    eps_rel=0.5e-3,
                    eps_abs=0.5e-3,
                )
            elif self.counter < 100:
                info_solv["full_setup"] = 1
                self.prog = osqp.OSQP()
                self.prog.setup(
                    P,
                    q,
                    A,
                    l,
                    u,
                    warm_start=True,
                    check_termination=25,
                    verbose=False,
                    polish=False,
                    eps_rel=0.5e-3,
                    eps_abs=0.5e-3,
                )
            else:
                info_solv["full_setup"] = 1
                self.prog = osqp.OSQP()
                self.prog.setup(
                    P,
                    q,
                    A,
                    l,
                    u,
                    warm_start=True,
                    check_termination=25,
                    verbose=False,
                    polish=False,
                    eps_rel=0.5e-3,
                    eps_abs=0.5e-3,
                )
                # self.prog.update(q=q, l=l, u=u)
                # self.prog.update(Px= P.data)
                # self.prog.update(Ax = A.data)

            self.prog.warm_start(x=self.warm_start)

            # dump needed when one wants to compile osqp
            # with open("param_dump.yaml", 'w') as dump:
            #    params = {'P': P, 'q': q, 'A': A, 'l': l, 'u': u}
            #    yaml.dump(params, dump)

            time2 = time.perf_counter()
            # print("Matrices setting time in s: {}".format(time2-time1))
            info_solv["set_matr_time"] = time2 - time1

            # and we solve
            time1 = time.perf_counter()
            res = self.prog.solve()
            time2 = time.perf_counter()
            # print("Solve time in s: {}".format(time2-time1))
            info_solv["solve_time"] = time2 - time1

            # print("solution")
            # print(res.x)

            # print constraint violation (hard constr)
            # print(F_constr)
            # print(f_constr)
            # print(res.x)
            # n_z = MPC_params['n_z']
            # viol = [(F_constr[2*i,:]@res.x[i*n_z:i*n_z+2] - f_constr[2*i], F_constr[2*i+1,:]@res.x[i*n_z:i*n_z+2] - f_constr[2*i+1]) for i in range(MPC_params['N'])]
            # print(viol)

            time1 = time.perf_counter()
            if res.info.status != "solved":
                print(res.info.status)
                # print(self.warm_start)
                inputs = self.prev_input
                self.warm_start = MPC_utils.update_warm_start_safe(
                    inputs,
                    self.warm_start,
                    self.car_params,
                    track=self.track,
                    slow_threshold=self.slow_threshold,
                )
                # raise ValueError('OSQP did not solve the problem!')
            else:
                self.warm_start = MPC_utils.update_warm_start(
                    res.x,
                    self.car_params,
                    self.track,
                    slow_threshold=self.slow_threshold,
                )
                inputs = res.x[self.MPC_params["n_x"] : self.MPC_params["n_z"]]
                self.prev_input = inputs
            time2 = time.perf_counter()
            # print("Warmstart update time in s: {}".format(time2-time1))
            info_solv["ws_update_time"] = time2 - time1

            info_solv["solve_status"] = res.info.status_val

            return inputs, F_constr, f_constr, max_P_el, info_solv

        elif solver == "forces":
            n_z = MPC_params["n_z"]
            n_x = MPC_params["n_x"]

            time1 = time.perf_counter()
            params, F_constr, f_constr, max_A_el = MPC_utils.get_forces_params(
                self.prev_input,
                X,
                U,
                Z_hist,
                self.track,
                self.track_hard,
                slow_threshold=self.slow_threshold,
            )
            time2 = time.perf_counter()
            print("    Setup time in s: {}".format(time2-time1))
            sol, exit_flag, info = self.forces_solver.solve(params)
            
            #print(info.dgap)
            print(exit_flag)
            info_solv = {"solve_status": exit_flag}

            #check_theta = (
            #    lambda x: x > MPC_params["T_s"] * MPC_params["upper_bound"]["v"]
            #)
            # sol_bad = np.array([check_theta(th) for th in (- sol['X'][Index.THETA+n_z::n_z] + sol['X'][Index.THETA:-n_z:n_z])]).any()
            # print(sol_bad)

            time1 = time.perf_counter()
            if exit_flag < 0:
                # print(params)
                # with open("./error_log/param_dump.yaml", 'w') as dump:
                #     yaml.dump(params, dump)
                # with open("./error_log/sol_dump.yaml", 'w') as dump:
                #     yaml.dump(sol, dump)
                # raise ValueError()
                inputs = np.array([0, 0.1])  # self.prev_input
                # self.warm_start = MPC_utils.get_initial_warmstart(
                #     N = MPC_params["N"],
                #     v_x0 = self.warm_start[Index.V],
                #     MPC_params = MPC_params,
                #     track = self.track,
                #     start_point = np.concatenate((self.warm_start[:Index.S_Y+1], self.warm_start[Index.PHI]), axis=None),
                #     start_theta = self.warm_start[Index.THETA],
                # )
                self.warm_start = MPC_utils.update_warm_start_safe(
                    inputs,
                    self.warm_start,
                    self.car_params,
                    track=self.track,
                    slow_threshold=self.slow_threshold,
                )
                accs = np.zeros(MPC_params["N"]+1)
                stvs = 0.1*np.ones(MPC_params["N"]+1)
                x_trajs = self.warm_start[Index.S_X::n_z]
                y_trajs = self.warm_start[Index.S_Y::n_z]
            else:
                accs = sol["X"][Index.ACC::n_z]
                stvs = sol["X"][Index.V_DELTA::n_z]
                x_trajs = sol["X"][Index.S_X::n_z]
                y_trajs = sol["X"][Index.S_Y::n_z]

                self.warm_start = MPC_utils.update_warm_start(
                    sol["X"],
                    self.car_params,
                    self.track,
                    slow_threshold=self.slow_threshold,
                )
                inputs = sol["X"][Index.V_DELTA : n_z]
                inputs = np.clip(
                    inputs,
                    [
                        MPC_params["lower_bound"]["v_delta"],
                        MPC_params["lower_bound"]["acc"],
                    ],
                    [
                        MPC_params["upper_bound"]["v_delta"],
                        MPC_params["upper_bound"]["acc"],
                    ],
                )
                self.prev_input = inputs
            # print(inputs)
            # print(self.warm_start)
            time2 = time.perf_counter()
            print("    Warmstart update time {}".format(time2-time1))

            return inputs, F_constr, f_constr, max_A_el, info_solv, accs, stvs, x_trajs, y_trajs

        elif solver == "nlp_forces":
            problem_params, use_slow = MPC_utils.get_nlp_params(X, U, self.warm_start, self.track, self.track_hard)

            if use_slow:
                output, exit_flag, info = self.nlp_solver_slow.solve(problem_params)
            else:
                output, exit_flag, info = self.nlp_solver_fast.solve(problem_params)
            
            # full_filename = forcespro.dump.save_problem(problem_params, "wawaweewa", '.')

            output_arr = [el for _, el in output.items()]
            info_solv = {'forces': info}
            info_solv['exit_flag'] = exit_flag
            print(exit_flag)
            if exit_flag <0:
                inputs = np.array([0, 0.1])  # self.prev_input
                self.warm_start = MPC_utils.update_warm_start_safe(
                    inputs,
                    self.warm_start,
                    self.car_params,
                    track=self.track,
                    slow_threshold=self.slow_threshold,
                )
            else:
                inputs = output["x01"][NlIndex.V_DELTA :NlIndex.ACC+1]
                inputs = np.clip(
                    inputs,
                    [
                        MPC_params["lower_bound"]["v_delta"],
                        MPC_params["lower_bound"]["acc"],
                    ],
                    [
                        MPC_params["upper_bound"]["v_delta"],
                        MPC_params["upper_bound"]["acc"],
                    ],
                )
                self.warm_start = MPC_utils.update_warm_start(
                    MPC_utils.ind_changer(np.concatenate(output_arr, axis=0), mode="nl2norm"),
                    self.car_params,
                    self.track,
                    slow_threshold=self.slow_threshold,
                )
            self.prev_input = inputs

            return inputs, None, None, 0, info_solv

        else:
            raise ValueError("Given solver ({}) is not supported.".format(solver))

    def ws_is_valid(self, state: np.array, threshold: float = 0.5) -> bool:
        """
        Returns a flag signalling whether the warm start is valid or not according to the current state
        """
        return (
            np.linalg.norm(state[: Index.BETA] - self.warm_start[: Index.BETA])
            <= threshold
        )
