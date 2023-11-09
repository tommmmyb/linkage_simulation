'''
author: tommmmyb
Simulation of Klann 6-bar walking mechanism, flipped vertically to be used to traverse monkey-bars
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from linkages import Klann
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


params = dict(

    ########################### common input parameters
    cycles = 4,
    input_angular_velocity = 1,
    ##########################

) | (geometry if not mg else dict())


def kinematic_sim():
    kwargs = params | (read_klann_specs('motiongen/klann_mechanism.pdf') if mg else dict()) | \
        dict(

        ############################### input parameters
        cycles = 1, # overwrite params
        grab = 0.0,
        release = 0.38*2*np.pi,
        input_angle = np.deg2rad(210),
        paths = [-1,8]
        ###############################

    )
    sys = Klann(**kwargs)
    
    fig, ax = plt.subplot_mosaic('ABB;ACC', constrained_layout=False, figsize=(15, 10))
    ax['B'].sharex(ax['C'])
    ax['B'].tick_params(labelbottom=False)
    ax['A'].set_prop_cycle('color', plt.cm.winter(np.linspace(.8,0,8)))
    ax['B'].set_prop_cycle('color', plt.cm.autumn([.1, .6]))
    ax['C'].set_prop_cycle('color', plt.cm.summer([.2, .7]))

    ax['A'].set_xlabel(r'x $[m]$')
    ax['A'].set_ylabel(r'y $[m]$')
    ax['B'].set_ylabel(r'velocity $[m/s]$')
    ax['C'].set_ylabel(r'acceleration $[m^{2}/s]$')
    ax['C'].set_xlabel(r'input crank angle $[deg]$')

    ax['A'].plot(sys.links[:, 8, 0], sys.links[:, 8, 1], label='hook path', linewidth=3); ax['A'].set_aspect('equal', adjustable='box')
    points = []
    for map in sys.connection_mapping.values():
         link_points, = ax['A'].plot(sys.links[0, map, 0], sys.links[0, map, 1], 'o-', linewidth=2)
         points.append(link_points)

    ax['B'].plot(np.rad2deg(sys.theta[:, 1]), sys.link_velocities[:, 8, 0], label='x-velocity', linewidth=2)
    ax['B'].plot(np.rad2deg(sys.theta[:, 1]), sys.link_velocities[:, 8, 1], label='y-velocity', linewidth=2)
    vline1 = ax['B'].axvline(np.rad2deg(sys.start_angle), color='black', linestyle='--')

    ax['C'].plot(np.rad2deg(sys.theta[:-1, 1]), sys.link_accelerations[:, 8, 0], label='x-acceleration', linewidth=2)
    ax['C'].plot(np.rad2deg(sys.theta[:-1, 1]), sys.link_accelerations[:, 8, 1], label='y-acceleration', linewidth=2)
    vline2 = ax['C'].axvline(np.rad2deg(sys.start_angle), color='black', linestyle='--')

    axslider = fig.add_axes([0.2, 0.9, 0.65, 0.02])
    slider = Slider(
        ax = axslider,
        label = 'input angle',
        valmin = 0,
        valmax = sys.num - 1,
        valinit = 0,
    )

    def update(val):
        [vline.set_xdata(np.array([np.rad2deg(sys.start_angle) + slider.val])) for vline in [vline1, vline2]]
        for link, map in sys.connection_mapping.items():
            points[link].set_data(sys.links[int(slider.val), map, 0], sys.links[int(slider.val), map, 1])

    slider.on_changed(update)
    fig.canvas.mpl_connect('key_press_event', lambda e: on_press(e, slider))
    [(a.grid(), a.legend(loc='lower left')) for a in ax.values()]
            
    plt.show()


def two_phase():
    kwargs = dict(

        ########################## input parameters
        paths = [-1,8],
        grab = np.pi + 0.35,
        release = 2*np.pi + 0.35
        ##########################

    ) | params| (read_klann_specs('motiongen/klann_mechanism.pdf') if mg else dict())
    # initialize linkage systems
    sys1 = Klann(input_angle=0, **kwargs)
    sys2 = Klann(input_angle=0, phase_shift=np.pi, origin=(0,0), **kwargs)

    systems = [sys1, sys2]

    # calculate movement of each linkage system relative to it's own hook
    for sys in systems:
        sys.move_body_rel_point(sys.links[:, [8], :], sys.mask)

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

    # plot initial state
    for sys in systems:
        c = ax['B']._get_lines.get_next_color()
        sys.plot_links(ax['B'], marker='o', color=c, linewidth=2, fill=False)
        sys.plot_paths(ax['B'], linestyle='-', color=c, linewidth=0.7)

    # add slider
    axslider = fig.add_axes([0.2, 0.9, 0.65, 0.02])
    slider = Slider(
        ax = axslider,
        label = 'input angle',
        valmin = 0,
        valmax = systems[0].num - 1,
        valinit = 0
    )
    # slider update function
    def update(val):
        for sys in systems:
            sys.update_links(int(slider.val))
            sys.update_paths(int(slider.val))

    slider.on_changed(update)
    fig.canvas.mpl_connect('key_press_event', lambda e: on_press(e, slider))

    plt.show()



def three_phase():
    kwargs = params | (read_klann_specs('motiongen/klann_mechanism.pdf') if mg else dict()) | \
        dict(

        ######################### input parameters
        paths = [8, -1],
        grab = 4/3*np.pi - 0.15,
        release = 2*np.pi - 0.15,
        theta0 = np.deg2rad(180)
        #########################

    )
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
    fig, ax = plt.subplot_mosaic('B', constrained_layout=False, figsize=(20, 5))
    ax['B'].set_prop_cycle('color', plt.cm.jet(np.linspace(0,1,6)))
    ax['B'].set_xlim(np.min(sys1.links[:,:,0]-0.025), np.max(sys2.links[:,:,0])+0.025), ax['B'].set_ylim(np.min(sys1.links[:,:,1]-0.025), np.max(sys2.links[:,:,1])+0.025)
    ax['B'].set_aspect('equal', adjustable=None)
    [a.grid() for a in ax.values()]
    

    # plot initial state
    for sys in systems:
        sys.plot_bars(ax['B'], 8, linestyle='', marker='o', color='black', markerfacecolor='none', markeredgewidth=3)
        c = ax['B']._get_lines.get_next_color()
        sys.plot_links(ax['B'], marker='o', color=c, linewidth=2, fill=False)
        sys.plot_paths(ax['B'], linestyle='-', color=c, linewidth=0.7)

    # this is terrible
    lines = ax['B'].get_lines()
    plt.setp(lines[-1], linestyle='--', color='black', linewidth=1.5)
    plt.setp(lines[7], linestyle='none')
    plt.setp(lines[15], linestyle='none')
    labels = ['monkey bars', 'Klann Linkage 1', 'Linkage 1 Hook Path', 'Klann Linkage 2', 'Linkage 2 Hook Path', 'Klann Linkage 3', 'Linkage 3 Hook Path', 'Ground Link Path']
    ax['B'].legend([lines[i] for i in [0,1,6,9,14,17,22,-1]], labels, loc='lower left')

    # add slider
    axslider = fig.add_axes([0.2, 0.9, 0.65, 0.02])
    slider = Slider(
        ax = axslider,
        label = 'input angle',
        valmin = 0,
        valmax = systems[0].num - 1,
        valinit = 0
    )
    # slider update function
    def update(val):
        for sys in systems:
            sys.update_links(int(slider.val))
            sys.update_paths(int(slider.val))

    slider.on_changed(update)
    fig.canvas.mpl_connect('key_press_event', lambda e: on_press(e, slider))

    plt.show()


# add keyboard interaction to slider
def on_press(event, slider):
    if event.key == 'right' and slider.val < slider.valmax:
        slider.set_val(slider.val + 3)
    if event.key == 'left' and slider.val > slider.valmin:
        slider.set_val(slider.val - 3)
    if event.key == 'shift+right' and slider.val < slider.valmax:
        slider.set_val(slider.val + 1)
    if event.key == 'shift+left' and slider.val > slider.valmin:
        slider.set_val(slider.val - 1)
    if event.key == 'ctrl+right' and slider.val < slider.valmax:
        slider.set_val(slider.val + 10)
    if event.key == 'ctrl+left' and slider.val > slider.valmin:
        slider.set_val(slider.val - 10)


if __name__ == '__main__':
    # kinematic_sim()
    # two_phase()
    three_phase()