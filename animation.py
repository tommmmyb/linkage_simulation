'''
author: tommmmyb
Animation of Klann 6-bar walking mechanism, flipped vertically to be used to traverse monkey-bars
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from linkages import Klann, Pantograph
try:
    from motiongen.read_motiongen import read_klann_specs
    mg = True
except ModuleNotFoundError as e:
    print(e)
    # use the following geometry if tabula library not found to read from motiongen file
    geometry = dict(
        L = [2.078, 1.016, 1.906, 1.196, 1.886, 1.789, 3.15, 2.251, 5.686],  # L's 0-8
        phi = np.deg2rad(32.5),                       # fixed acute angle between L2 & L5
        theta0 = np.pi,                               # fixed angle between grounds of 4-bar
        theta_constraint = np.deg2rad(-58),           # fixed angle between grounds of 5-bar
    )
    mg = False



def klann_three_phase():
    kwargs = dict(

        ######################### input parameters
        cycles = 4,
        input_angular_velocity = 1,
        paths = [-1,8],
        grab = 4/3*np.pi - 0.15,
        release = 2*np.pi - 0.15
        #########################

    ) | (geometry if not mg else dict()) | (read_klann_specs('motiongen/klann_mechanism.pdf') if mg else dict()) 
    # initialize linkage systems
    sys1 = Klann(input_angle=0, **kwargs)
    sys2 = Klann(input_angle=0, phase_shift=2/3*np.pi, origin=(0,0), **kwargs)
    sys3 = Klann(input_angle=0, phase_shift=4/3*np.pi, origin=(0,0), **kwargs)

    systems = [sys1, sys2, sys3]

    # calculate movement of each linkage system relative to it's own hook
    for sys in systems:
        sys.move_body_rel_point(sys.links[:, [8], :], sys.mask)

    # calculate movement of each linkage system relative to the other two systems
    sys1.move_body_rel_point(-1*sys2.links[:, -1:, :], sys2.mask)
    sys1.move_body_rel_point(-1*sys3.links[:, -1:, :], sys3.mask)

    sys2.move_body_rel_point(-1*sys1.links[:, -1:, :], sys1.mask)
    sys2.move_body_rel_point(-1*sys3.links[:, -1:, :], sys3.mask)

    sys3.move_body_rel_point(-1*sys1.links[:, -1:, :], sys1.mask)
    sys3.move_body_rel_point(-1*sys2.links[:, -1:, :], sys2.mask)

    # shift the origin (ground point) of each linkage system at each release point
    sys1.update_origins(sys1.merged_release_mask([sys2.mask, sys3.mask]))
    sys2.update_origins(sys2.merged_release_mask([sys1.mask, sys3.mask]))
    sys3.update_origins(sys3.merged_release_mask([sys1.mask, sys2.mask]))


    # plot setup
    fig, ax = plt.subplot_mosaic('B', constrained_layout=True, figsize=(20, 5))
    ax['B'].set_prop_cycle('color', plt.cm.jet(np.linspace(0,1,6)))
    ax['B'].set_xlim(np.min(sys1.links[:,:,0]-0.25), np.max(sys2.links[:,:,0])+0.25), ax['B'].set_ylim(np.min(sys1.links[:,:,1]-0.25), np.max(sys2.links[:,:,1])+0.25)
    ax['B'].set_aspect('equal', adjustable=None)
    [a.grid() for a in ax.values()]

    artists = []

    # plot initial state
    def init():
        for sys in systems:
            c = ax['B']._get_lines.get_next_color()
            sys.plot_links(ax['B'], marker='o', color=c, linewidth=2, fill=False)
            sys.plot_paths(ax['B'], linestyle='-', color=c, linewidth=0.7)
            artists.extend(sys.link_plots)
            artists.extend(sys.path_plots)
        return artists,

    def update(frame):
        for sys in systems:
            sys.update_links(int(frame))
            sys.update_paths(int(frame))
        return artists,


    ani = FuncAnimation(fig, update, frames=np.linspace(0, systems[0].num - 1, systems[0].num), interval=1,
                    init_func=init, blit=False)


    # plt.show()
    ani.save('animations/klann.gif', PillowWriter(fps=30))


def pantograph_two_phase():
    kwargs = dict(
        cycles = 4,
        L = [2.75, 0.759, 1.14, 2.92, 2.55, 7.17],    # L's 0-5
        phi = 0.185,                                  # [rad] fixed acute angle between L4 & L5
        input_angular_velocity = 1,
        theta0 = np.deg2rad(0),
        theta_constraint = np.deg2rad(-38),
        paths = [-1,5],
        grab = 2*np.pi + 0.65,
        release = np.pi + 0.65
    )
    # initialize linkage systems
    sys1 = Pantograph(input_angle=0, **kwargs)
    sys2 = Pantograph(input_angle=0, phase_shift=np.pi, origin=(0,0), **kwargs)

    systems = [sys1, sys2]

    # calculate movement of each linkage system relative to it's own hook
    for sys in systems:
        sys.move_body_rel_point(sys.links[:, [5], :], sys.mask)

    # calculate movement of each linkage system relative to the other two systems
    sys1.move_body_rel_point(-1*sys2.links[:, -1:, :], sys2.mask)
    sys2.move_body_rel_point(-1*sys1.links[:, -1:, :], sys1.mask)


    # shift the origin (ground point) of each linkage system at each release point
    sys1.update_origins(sys1.merged_release_mask([sys2.mask]))
    sys2.update_origins(sys2.merged_release_mask([sys1.mask]))



    # plot setup
    fig, ax = plt.subplot_mosaic('B', constrained_layout=True, figsize=(20, 5))
    ax['B'].set_prop_cycle('color', plt.cm.jet(np.linspace(0,1,6)))
    ax['B'].set_xlim(np.min(sys1.links[:,:,0]-0.25), np.max(sys2.links[:,:,0])+0.25), ax['B'].set_ylim(np.min(sys1.links[:,:,1]-0.25), np.max(sys2.links[:,:,1])+0.25)
    ax['B'].set_aspect('equal', adjustable=None)
    [a.grid() for a in ax.values()]

    artists = []

    # plot initial state
    def init():
        for sys in systems:
            c = ax['B']._get_lines.get_next_color()
            sys.plot_links(ax['B'], marker='o', color=c, linewidth=2, fill=False)
            sys.plot_paths(ax['B'], linestyle='-', color=c, linewidth=0.7)
            artists.extend(sys.link_plots)
            artists.extend(sys.path_plots)
        return artists,

    def update(frame):
        for sys in systems:
            sys.update_links(int(frame))
            sys.update_paths(int(frame))
        return artists,


    ani = FuncAnimation(fig, update, frames=np.linspace(0, systems[0].num - 1, systems[0].num), interval=1,
                    init_func=init, blit=False)


    # plt.show()
    ani.save('animations/pantograph.gif', PillowWriter(fps=30))


if __name__ == '__main__':
    # klann_three_phase()
    pantograph_two_phase()