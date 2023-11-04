'''
author: tommmmyb
Simulation of dual-driven pantograph 5-bar mechanism to be used to traverse monkey-bars
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from linkages import Pantograph


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


def two_phase():
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
    kwargs = dict(
        cycles = 4,
        L = [2.75, 0.759, 1.14, 2.92, 2.55, 7.17],    # L's 0-5
        phi = 0.185,                                  # [rad] fixed acute angle between L4 & L5
        input_angular_velocity = 1,
        theta0 = np.deg2rad(0),
        theta_constraint = np.deg2rad(-38),
        paths = [-1,5],
        grab = 2/3*np.pi - 0.9,
        release = 4/3*np.pi - 0.9
    )
    # initialize linkage systems
    sys1 = Pantograph(input_angle=0, **kwargs)
    sys2 = Pantograph(input_angle=0, phase_shift=2/3*np.pi, origin=(0,0), **kwargs)
    sys3 = Pantograph(input_angle=0, phase_shift=4/3*np.pi, origin=(0,0), **kwargs)

    systems = [sys1, sys2, sys3]

    # calculate movement of each linkage system relative to it's own hook
    for sys in systems:
        sys.move_body_rel_point(sys.links[:, [5], :], sys.mask)

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


if __name__ == '__main__':
    two_phase()
    # three_phase()