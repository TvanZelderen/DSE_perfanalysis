import numpy as np
import os
from matplotlib import pyplot as plt
import csv

##### This code is made with the help of ChatGPT ####
read_directory = "D:/Downloads/xflr5_6.57_win64/xflr5_6.57_win64/airfoil_result"

output_directory = 'C:/Users/25445/Documents/GitHub/DSE_perfanalysis/airfoil_database'

for filename in os.listdir(read_directory):
    input_file_path = os.path.join(read_directory, filename)
    output_file_path = os.path.join(output_directory, filename.replace('.txt', '.csv'))
    with open(input_file_path, 'r') as txt_file, open(output_file_path, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        for line in txt_file:
        # Split the line into columns (adjust delimiter as needed)
            columns = line.split()  # Default splits by any whitespace
            csv_writer.writerow(columns)
