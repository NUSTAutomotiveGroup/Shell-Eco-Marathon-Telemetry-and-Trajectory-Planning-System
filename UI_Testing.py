import matplotlib.pyplot as plt
import matplotlib.colors as colors
import numpy as np
from matplotlib import cm
import matplotlib.cbook as cbook
import csv
from matplotlib.colors import LinearSegmentedColormap, ListedColormap

x = []
y = []
Z = []

with open('sem_apme_2025-track_coordinates.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',')
    next(csvreader, None)
    for row in csvreader:
        x.append(float(row[1]))
        y.append(float(row[2]))
        Z.append(int(float(row[3]) * 10 - 99))

fig, ax = plt.subplots(1, 1)
n = 60
z = np.array(Z)

colors = plt.cm.jet(np.linspace(0, 1, n))

scatter = plt.scatter(x, y, label='Lusail Coordinates', color = colors[z])
plt.colorbar(scatter, ax=ax, label='Elevation (m)')
plt.gca().invert_xaxis()


plt.title('Lusail Coordinates Plot')
plt.grid()
plt.legend()
plt.show()
 