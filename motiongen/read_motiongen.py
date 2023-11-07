from tabula import read_pdf
from numpy import pi

def read_klann_specs(file):
    df = read_pdf(file, stream=True, pages=3)[0]
    df.set_index('Linked Joints', inplace=True)
    
    links = [
        'J1 - J2',        # L0
        'J1 - J4',        # L1
        'J4 - J5',        # L2
        'J5 - J2',        # L3
        'J2 - J3',        # L4
        'J7 - J5',        # L5
        'J6 - J3',        # L6
        'J7 - J6',        # L7
        'J6 - J8'         # L8
    ]

    L = df['Length [cm]'][links].to_numpy()/100 # convert to meters
    
    theta0 = df['Angle [rad]']['J1 - J2'].astype(float)
    theta_constraint = df['Angle [rad]']['J2 - J3'].astype(float)
    phi = ( df['Angle [rad]']['J7 - J5'].astype(float) - df['Angle [rad]']['J4 - J5'].astype(float) - pi ) * -1

    return dict(L = L, theta0 = theta0, theta_constraint = theta_constraint, phi = phi)
