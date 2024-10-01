import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.patches import Polygon
import sys

colors = ["green3", "DimGrey", "DodgerBlue3", "orange", "salmon", "peru", "khaki3", "brown", "black", "DarkOliveGreen", "sienna", "tomato2", "IndianRed", "SkyBlue", "cyan3"]

if len(sys.argv) < 5:
  print("python plot_tour.py TOUR_FILE MAP_FILE TARGETS_FILE OUT_IMAGE_FILE")

pts = []
p_col = []
lines = []
tris = []
lim_x = [float('inf'), float('-inf')]
lim_y = [float('inf'), float('-inf')]

color_index = 0

ax = plt.gca()
#ax.set_title(sys.argv[1])

############ Read the tour ##################
nodes = open(sys.argv[1], 'r')

for line in nodes:
  # only for 2D tours
  ar = list(map(float, line.strip().split(' ')))
  px = ar[0]
  py = ar[1]
  color = ar[-1]
  
  lim_x[0] = min(lim_x[0], px)
  lim_x[1] = max(lim_x[1], px)
  
  lim_y[0] = min(lim_y[0], py)
  lim_y[1] = max(lim_y[1], py)

  pts.append([ar[0], ar[1]])

  p_col.append(int(ar[-1]))

np_pts = np.asarray(pts)
np_col = np.asarray(p_col)

############ Read the map ##################

triangle_file = open(sys.argv[2], 'r')
triangles = []

for line in triangle_file:
  iter = 0
  ar = list(map(float, line.strip().split(' ')))
  for pt in ar:
    if iter % 2 == 0:
      lim_x[0] = min(lim_x[0], pt)
      lim_x[1] = max(lim_x[1], pt)
    elif iter % 2 == 1:
      lim_y[0] = min(lim_y[0], pt)
      lim_y[1] = max(lim_y[1], pt)
    
    iter += 1
  loc_pts = np.array([[ar[0], ar[1]], [ar[2], ar[3]], [ar[4], ar[5]]])
  triangles.append(Polygon(loc_pts, fill=True, color='royalblue'))
  
############ Read the targets ###############
target_file = open(sys.argv[3], 'r')
targets = []

for line in target_file:
  if len(line) < 3:
    break
  ar = list(map(float, line.strip().split('\t')))
  px = ar[0]
  py = ar[1]
  
  lim_x[0] = min(lim_x[0], px)
  lim_x[1] = max(lim_x[1], px)
  
  lim_y[0] = min(lim_y[0], py)
  lim_y[1] = max(lim_y[1], py)

  targets.append([ar[0], ar[1]])

np_targets = np.asarray(targets)
############ Manage limits #################
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)
ax.set_xlim(lim_x[0], lim_x[1])
ax.set_ylim(lim_y[0], lim_y[1])


# Plot map
for triangle in triangles:
  ax.add_patch(triangle)

# Plot tour
ax.scatter(np_pts[:, 0], np_pts[:, 1], c=np_col, s=10)

# Plot targets
ax.scatter(np_targets[:, 0], np_targets[:, 1], c="black", s=10)

# Save to the last file
plt.savefig(sys.argv[4])
#plt.show()