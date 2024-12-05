import numpy as np
from airfoil import f_airfoil
from matplotlib import pyplot as plt


r_landzone = 500
x_landzone = []
y_landzone = []
theta = np.arange(0, 6.28, 0.01)
for element in theta:
    # print(x)
    x_landzone.append(np.cos(element) * r_landzone)
    y_landzone.append(np.sin(element) * r_landzone)

print(len(y_landzone), len(x_landzone))

plt.figure()
plt.plot(x_landzone, y_landzone)
plt.show()