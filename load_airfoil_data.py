import numpy as np
import os
from matplotlib import pyplot as plt
import csv
from pathlib import Path

##### This code is made with the help of ChatGPT ####
read_directory = Path("airfoil_input")

output_directory = Path("airfoil_database")

for filename in os.listdir(read_directory):
    input_file_path = os.path.join(read_directory, filename)
    output_file_path = os.path.join(output_directory, filename.replace('.txt', '.csv'))
    with open(input_file_path, 'r') as txt_file, open(output_file_path, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        for _ in range(11):
            next(txt_file)  # This reads and discards the first 11 lines
        for line in txt_file:
        # Split the line into columns (adjust delimiter as needed)
            columns = line.split()  # Default splits by any whitespace
            csv_writer.writerow(columns)

