import numpy as np
from matplotlib import pyplot as plt

def launch_vehicle_drag_coef(mach):
    if mach < 0:
        raise ValueError("The mach number has to be above 0.")
    if mach <= 0.6:
        return 0.2083333 * mach ** 2 - 0.25 * mach + 0.46
    if mach <= 0.8:
        return 1.25 * mach ** 3 - 2.125 * mach ** 2 + 1.2 * mach + 0.16
    if mach <= 0.95:
        return 10.37037 * mach ** 3 - 22.88889 * mach ** 2 + 16.91111 * mach - 3.78963
    if mach <= 1.05:
        return - 30 * mach ** 3 + 88.5 * mach ** 2 - 85.425 * mach + 27.51375
    if mach <= 1.15:
        return - 20 * mach ** 3 + 60 * mach ** 2 - 58.65 * mach + 19.245
    if mach <= 1.3:
        return 11.85185 * mach ** 3 - 44.88889 * mach ** 2 + 56.22222 * mach - 22.58519
    if mach <= 2:
        return - 0.04373178 * mach ** 3 + 0.3236152 * mach ** 2 - 1.019679 * mach + 1.554752
    if mach <= 3.25:
        return 0.01024 * mach ** 3 - 0.00864 * mach ** 2 - 0.33832 * mach + 1.08928
    if mach <= 4.5:
        return - 0.01408 * mach ** 3 + 0.19168 * mach ** 2 - 0.86976 * mach + 1.53544
    if mach > 4.5:
        return 0.22
    else:
        raise ValueError()
    
if __name__ == "__main__":
    print(launch_vehicle_drag_coef(1))
    
    macharr = np.arange(0, 4.5, 0.01)
    cdarr = np.array([])

    for element in macharr:
        # print(element)
        cd = launch_vehicle_drag_coef(element)
        cdarr = np.append(cdarr, cd)

    plt.figure()
    plt.plot(macharr, cdarr)
    plt.show()