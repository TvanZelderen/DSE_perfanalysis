import numpy as np
import pandas as pd

# Create grid of parameters
altitudes = np.arange(0, 28000, 1000)
velocities = [30, 50, 75, 100, 150, 200, 250, 300, 350, 400]
aoa_range = np.linspace(-7, 15, 23)

# Create multi-index DataFrame
index = pd.MultiIndex.from_product([
    altitudes, 
    velocities, 
    aoa_range
], names=['Altitude', 'Velocity', 'Angle_of_Attack'])

df = pd.DataFrame(index=index, columns=['CL', 'CD', 'Cm'])

# Save template for manual filling
df.to_csv('xflr5_data_template.csv')