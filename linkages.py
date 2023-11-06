'''
author: tommmmyb
klann and pantograph object classes
'''

import numpy as np
from scipy.optimize import root
from matplotlib.patches import Polygon

class LinkageSystem():
    def __init__(self, 
                cycles: int,
                L: list, 
                input_angle: float, 
                input_angular_velocity: float, 
                theta0: float,
                theta_constraint: float, 
                phi: float,
                grab: float,
                release: float,
                paths: list,
                phase_shift = 0.0,
                origin = (0,0),
                ) -> None:
        # geometric inputs
        self.cycles = cycles
        self.range = cycles * 2*np.pi
        self.num_per_cycle = 360
        self.num = cycles * self.num_per_cycle
        self.origin = np.array(origin)
        self.grab = ( grab - phase_shift ) % (2*np.pi)
        self.release = ( release - phase_shift ) % (2*np.pi)
        self.phase_shift = phase_shift
        self.paths = paths
        self.L = np.array(L)
        self.phi = phi
        self.start_angle = input_angle + phase_shift
        self.theta_constraint = theta_constraint

        # initialize theta & w matrices
        self.theta = np.zeros((self.num, len(L)))
        self.w = np.zeros((self.num, len(L)))

        # initialize plot artists
        self.link_plots = []
        self.path_plots = []

        # known angles/angular vel
        self.theta[:, 0] = theta0                                                                       # fixed angle between ground1 and ground2
        self.theta[:, 1] = np.linspace(self.start_angle, self.start_angle + self.range, num=self.num)   # input angles of driven link, L1
        self.w[:, 1] = np.ones(self.num) * input_angular_velocity                                       # input angular velocity of driven link, L1

        # solve for theta, w
        self.position()
        self.velocity()

        self.mapping_setup()
        
        # final joint/link kinematics
        self.link_pos()              # solve for x,y relative to body fixed origin (origin addes as last column)
        self.create_grab_release_mask()


    def rad2index(self, val):
        return val / (2*np.pi) * self.num_per_cycle
    
    def link_pos(self):
        cos_sin = np.stack([np.cos(self.theta), np.sin(self.theta)], axis=2)
        xy = np.einsum('i,jik->jik', self.L, cos_sin)

        self.joints = np.zeros(xy.shape)

        for link, map in self.mapping.items():
            self.joints[:,link,:] = np.sum(xy[:,map,:], axis=1) + self.origin

        self.links = np.append(self.joints, (np.ones((self.joints.shape[0], 1, self.joints.shape[2])) * self.origin), axis=1) # add origin at end of joint array

    def create_grab_release_mask(self):
        mask = np.zeros(self.num_per_cycle, dtype=int)
        g = round(self.rad2index(self.grab))
        r = round(self.rad2index(self.release))
        if g < r:
            mask[g:r] = 1
            mask[r-1] = -1
            mask[g] = 2
        elif r < g:
            mask[:r] = 1
            mask[r-1] = -1
            mask[g:] = 1
            mask[g] = 2
        self.mask = np.tile(mask, self.cycles)

    def move_body_rel_point(self, point: np.ndarray, mask: np.ndarray):
        # point should be Klann().links[:, [#], :], so that shape = (n, 1, 2)
        # mask should be 1D array of length num*cycles (i.e. every sample for the simulation)
        # "body" refers to all points of instance of Klann(), (may include "point" if "point" is w/in the same body, i.e. "point" will remain stationary)
        
        # TODO: fix endpoint bug, i.e. when release (grab too?) falls directly on the last sample (first sample too?), motion simulation breaks
        # leaving this here in case mask indexing changes to deal with endpoint bug or structural change
        # if mask[0] == 0:
        #     mask[0] = -1       # set inital index to a release point
        # elif mask[0] == 1:
        #     mask[0] = 2       # set inital index to a grab point
        
        grabbed_ind = mask.nonzero()[0]
        grabbed_mask = mask[grabbed_ind]
        grabbed_mask[0] = 2
        grabbed_points = point[grabbed_ind, :, :]
        grabbed_start_vals = grabbed_points[np.where(grabbed_mask == 2)[0], :, :]
        chunks = np.split(grabbed_mask, np.where(grabbed_mask == 2)[0])
        chunk_lengths = np.array([chunk.shape[0] for chunk in chunks])
        adjustment = np.repeat(grabbed_start_vals, chunk_lengths[1:], axis=0)
        normalized_path = grabbed_points - adjustment

        self.links[grabbed_ind, :, :] -= normalized_path

    # should probably do this in a more logical way, but this works
    def merged_release_mask(self, masks: list):
        masks = self.mask + masks
        new_mask = np.zeros(self.num)
        for mask in masks:
            new_mask[np.where(mask == -1)[0]] = -1
        return new_mask

    def update_origins(self, mask):
        release_coords = self.links[np.where(mask == -1)[0], -1:, :] # x,y positions of new origins at each release point (origin must be in last column of links array)
        chunks = np.split(mask, np.where(mask == -1)[0])
        chunk_lengths = np.array([chunk.shape[0] for chunk in chunks])
        release_coords = np.insert(release_coords, 0, self.origin, axis=0)
        origin_corrections = np.repeat(np.cumsum(release_coords - self.origin, axis=0), chunk_lengths, axis=0)

        self.links[1:,:,:] += origin_corrections[:-1,:,:]

    def movement_sim(self, link):
        self.move_body_rel_point(point=self.links[:, [link], :], mask=self.mask)
        self.update_origins(self.mask)
        
    def plot_links(self, ax, fill=False, **kwargs):
        for map in self.connection_mapping.values():
            if len(set(map)) != len(map) and fill:
                line, = ax.fill(self.links[0, map, 0], self.links[0, map, 1])
            else:
                line, = ax.plot(self.links[0, map, 0], self.links[0, map, 1], **kwargs)
            self.link_plots.append(line)
    
    def plot_paths(self, ax, **kwargs):
        for joint in self.paths:
            line, = ax.plot(*np.split(self.links[:0, joint, :], 2, axis=-1), **kwargs)
            self.path_plots.append(line)

    def update_links(self, index):
        for line, map in zip(self.link_plots, self.connection_mapping.values()):
            if isinstance(line, Polygon):
                line.set_xy(self.links[index, map, :])
            else:
                line.set_data(self.links[index, map, 0], self.links[index, map, 1])

    def update_paths(self, index):
        for line, joint in zip(self.path_plots, self.paths):
            line.set_data(*np.split(self.links[:index, joint, :], 2, axis=-1))







class Klann(LinkageSystem):
    def __init__(self, cycles: int, L: list, input_angle: float, input_angular_velocity: float, theta0: float, theta_constraint: float, phi: float, grab: float, release: float, paths: list, phase_shift=0, origin=(0, 0)) -> None:
        super().__init__(cycles, L, input_angle, input_angular_velocity, theta0, theta_constraint, phi, grab, release, paths, phase_shift, origin)
        # theta_constraint is theta4, the acute angle between L0 and L4
        # phi is the acute angle between L2 and L5

    def mapping_setup(self):
        self.mapping = {
            0: [0],
            1: [1],
            2: [1, 2],
            3: [0, 3],
            4: [0, 4],
            5: [0, 3, 5],
            6: [0, 4, 6],
            7: [0, 4, 6, 7],
            8: [0, 4, 6, 8]
        }
        self.connection_mapping = {
            0: [9, 1],
            1: [0, 3],
            2: [1, 3, 5, 1],
            3: [4, 6],
            4: [4, 6, 8],
        }

    def position(self):
        # klann specific constraint
        self.theta[:, 4] = self.theta_constraint        # fixed angle between grounds of 5-bar
        self.fourbar_pos()                              # solve for theta2 & theta3, update theta matrix
        self.theta[:, 5] = self.theta[:, 2] - self.phi  # apply ternary link position constraint
        self.fivebar_pos()                              # solve for theta6 & theta7, update theta matrix
        self.theta[:, 8] = self.theta[:, 7]             # set arm angle equal to link7 angle (link7 is colinear w/ the arm)

    def velocity(self):
        self.fourbar_vel()                        # solve for w2 & w3, update w matrix
        self.w[:, 5] = self.w[:, 2]               # apply ternary link velocity constraint
        self.fivebar_vel()                        # solve for w6 & w7
        self.w[:, 7] = self.w[:, 8]               # w7 = w8 (colinear link7 & link8)
        
    def fourbar_pos(self):
        # fourbar position analysis using half-angle method
        # solves assuming theta[0]=0, then adds inputted theta[0] to rotate entire linkage system
        K1 = self.L[0]/self.L[1]
        K2 = self.L[0]/self.L[3]
        K3 = (self.L[1]**2 - self.L[2]**2 + self.L[3]**2 + self.L[0]**2) / (2*self.L[1]*self.L[3])
        K4 = self.L[0]/self.L[2]
        K5 = (self.L[3]**2 - self.L[0]**2 - self.L[1]**2 - self.L[2]**2) / (2*self.L[1]*self.L[2])
        Q = np.cos(self.theta[:, 1] - self.theta[:, 0])
        A = K3 - K1 - (K2 - 1)*Q
        B = -2*np.sin(self.theta[:, 1] - self.theta[:, 0])
        C = K3 + K1 - (K2 + 1)*Q
        D = K5 - K1 + (K4 + 1)*Q
        E = B
        F = K5 + K1 + (K4 - 1)*Q
        self.theta[:, 2] = 2*np.arctan2(-E-np.sqrt(E**2 - 4*D*F), 2*D) + self.theta[:, 0]
        self.theta[:, 3] = 2*np.arctan2(-B-np.sqrt(B**2 - 4*A*C), 2*A) + self.theta[:, 0]


    def fourbar_vel(self):
        # standard fourbar velocity equations
        self.w[:, 2] = -self.w[:, 1] * (self.L[1]/self.L[2]) * np.sin(self.theta[:, 1] - self.theta[:, 3]) / np.sin(self.theta[:, 2] - self.theta[:, 3])
        self.w[:, 3] =  self.w[:, 1] * (self.L[1]/self.L[3]) * np.sin(self.theta[:, 2] - self.theta[:, 1]) / np.sin(self.theta[:, 2] - self.theta[:, 3])

    def fivebar_pos(self):
        # solve system of nonlinear equations for 5-bar position using scipy.optimize.root
        def funcs(guesses: list, i: int): 
            return [
            self.L[3]*np.cos(self.theta[i, 3]) + self.L[5]*np.cos(self.theta[i, 5]) - self.L[4]*np.cos(self.theta[i, 4]) - self.L[6]*np.cos(guesses[0]) - self.L[7]*np.cos(guesses[1]),
            self.L[3]*np.sin(self.theta[i, 3]) + self.L[5]*np.sin(self.theta[i, 5]) - self.L[4]*np.sin(self.theta[i, 4]) - self.L[6]*np.sin(guesses[0]) - self.L[7]*np.sin(guesses[1])
        ]
        guesses = [np.pi, np.pi/4] # initial guesses for when theta[:, 1] = 0
        for i in range(self.theta.shape[0]):
            self.theta[i, 6], self.theta[i, 7] = root(funcs, guesses, args=(i,)).x
            guesses = [self.theta[i, 6], self.theta[i, 7]]

    def fivebar_vel(self):
        # solve system of nonlinear equations for 5-bar velocity using scipy.optimize.root
        def funcs(guesses: list, i: int): 
            return [
            self.L[3]*self.w[i, 3]*np.cos(self.theta[i, 3]) + self.L[5]*self.w[i, 5]*np.cos(self.theta[i, 5]) - self.L[6]*guesses[0]*np.cos(self.theta[i, 6]) - self.L[7]*guesses[1]*np.cos(self.theta[i, 7]),
            self.L[3]*self.w[i, 3]*np.sin(self.theta[i, 3]) + self.L[5]*self.w[i, 5]*np.sin(self.theta[i, 5]) - self.L[6]*guesses[0]*np.sin(self.theta[i, 6]) - self.L[7]*guesses[1]*np.sin(self.theta[i, 7])
        ]
        guesses = [0.5, 0.5] # initial guesses for when w[:, 1] = 1
        for i in range(self.w.shape[0]):
            self.w[i, 6], self.w[i, 7] = root(funcs, guesses, args=(i,)).x
            guesses = [self.w[i, 6], self.w[i, 7]]

    def hook_pos(self):
        # determine x,y coordinates of hook
        self.hook_x = self.L[6]*np.cos(self.theta[:, 6]) + self.L[8]*np.cos(self.theta[:, 8])
        self.hook_y = self.L[6]*np.sin(self.theta[:, 6]) + self.L[8]*np.sin(self.theta[:, 8])

    # this might be wrong, idk. it also doesn't account for moving bodies
    def hook_vel(self):
        self.hook_xvel = self.L[6]*self.w[:, 6]*np.sin(self.theta[:, 6]) + self.L[8]*self.w[:, 8]*np.sin(self.theta[:, 8])
        self.hook_yvel = self.L[6]*self.w[:, 6]*np.cos(self.theta[:, 6]) + self.L[8]*self.w[:, 8]*np.cos(self.theta[:, 8])

    def arm_pos(self):
        self.arm_x = np.array([self.hook_x, self.hook_x - self.L[8]*np.cos(self.theta[:, 8])])
        self.arm_y = np.array([self.hook_y, self.hook_y - self.L[8]*np.sin(self.theta[:, 8])])








class Pantograph(LinkageSystem):
    def __init__(self, cycles: int, L: list, input_angle: float, input_angular_velocity: float, theta0: float, theta_constraint: float, phi: float, grab: float, release: float, paths: list, phase_shift=0, origin=(0, 0)) -> None:
        super().__init__(cycles, L, input_angle, input_angular_velocity, theta0, theta_constraint, phi, grab, release, paths, phase_shift, origin)
        # theta_constraint is the difference between theta1 and theta2, which is the offset angle between the two geared links
        # phi is the angle between theta4 and theta5, which is the angle of the ternary link that interfaces with the driven link

    def mapping_setup(self):
        self.mapping = {
            0: [0],
            1: [1],
            2: [0, 2],
            3: [0, 2, 3],
            4: [1, 4],
            5: [1, 5],
        }
        self.connection_mapping = {
            0: [0, 2, 3],
            1: [-1, 1, 4],
            2: [1, 5, 4, 1],
        }

    def position(self):
        self.theta[:, 2] = self.theta[:, 1] + self.theta_constraint
        def funcs(guesses: list, i: int): 
            return [
                self.L[1] * np.cos(self.theta[i, 1]) + self.L[4] * np.cos(guesses[1]) - self.L[3] * np.cos(guesses[0]) - self.L[2] * np.cos(self.theta[i, 2]) - self.L[0] * np.cos(self.theta[i, 0]),
                self.L[1] * np.sin(self.theta[i, 1]) + self.L[4] * np.sin(guesses[1]) - self.L[3] * np.sin(guesses[0]) - self.L[2] * np.sin(self.theta[i, 2]) - self.L[0] * np.sin(self.theta[i, 0])
            ]
        
        # Guesses for solver (theta3, theta4)
        guesses = [3/4*np.pi, 1/4*np.pi]

        # Solve unknown angles
        for i in range(self.theta.shape[0]):
            self.theta[i, 3], self.theta[i, 4] = root(funcs, guesses, args=(i,)).x
            guesses = [self.theta[i, 3], self.theta[i, 4]]
        self.theta[:, 5] = self.theta[:, 4] + self.phi  # apply ternary link position constraint

    def velocity(self):
        # not implemented
        return










