#!/usr/bin/env python3

import csv



path = '/home/klein/ws/src/hack/config/' + 'challenge1_waypoints_01.csv'

#Read waypoints from CSV file and store in marker points
with open(path, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # Skip the header row
    for row in csvreader:
        # Extract the x, y, z values from the last three columns
        x, y, z = map(float, row[-3:])  # Convert the x,y,z to floats
        print(z)