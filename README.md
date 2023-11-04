# linkage_simulation
Simulates the kinematics of various linkage mechanisms for traversing monkey-bars

## usage
run klann_simulation.py or pantograph_simulation.py to generate interactive simulation plots

## requirements
`pip install -r requirements.txt`

- simulation utilizes numpy, scipy, and matplotlib
- reading data from motiongen files requires tabula-py (which depends on pandas)

## choosing simulations
scroll to the bottom of either simulation file. Under `if __name__ == '__main__'`, comment/uncomment the function calls to choose the desired output

## modifying geometry
the functions kinematics_sim(), two_phase(), and three_phase() each start with a dictionary which defines the geometric constants of the linkage system, for example:

```
kwargs = dict(
    cycles = 4,                                   # number of cycles, each cycle being 2*pi rotation of L1
    grab = 2/3*np.pi,                             # point along cycle where link attached to bar
    release = 4/3*np.pi,                          # point along cycle where link releases from bar
    L = [2.078, 1.016, 1.906, 1.196, 1.886, 1.789, 3.15, 2.251, 5.686],  # lengths 0-8
    phi = np.deg2rad(32.5),                       # fixed acute angle between L2 & L5
    input_angle = np.deg2rad(210),                # starting angle of input link, L1
    input_angular_velocity = 1,                   # constant input ang. vel. for velocity sim
    theta0 = np.pi,                               # fixed angle between ground points
    theta_constraint = np.deg2rad(-58),           # meaning varies for different linkage systems
    paths = [8, -1]                               # indices of the joints of which to trace the movement path (origin is the last column)
)
``` 

## meaning of `theta_constraint` and `phi`
- for the klann mechanism:
    - theta_constraint = theta4, as shown in `reference/variable_diagram.pdf`
    - phi is the fixed acute angle between L2 and L5, as shown in `reference/variable_diagram.pdf`
- for the pantograph:
    - theta_constraint is the difference in angle between L1 and L2, where L1 is driven and L2 is geared to L1
    - phi is the angle between L4 and L5, which is the angle of the corner of the ternary link that is connected to the driven link (L1)


## units
`reference/dimensions.pdf` and `motiongen/klann_mechanism` use millimeters, the default values in the python files are in inches (will be converted)

## motiongen.io integration
In the `motiongen` folder, there's a `.motiongen` file that can be opened by going to `motiongen.io` and importing the file. If you make changes and export to a pdf, then replace that pdf with the one currently in the `motiongen` folder, the simulation scripts will read in the new geometry. (Do not add/remove links in motiongen, if any of the link numbers change, the simulation scripts will break)