import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import sys

mapName = str(sys.argv[1])
gridName = str(sys.argv[2])
map = np.loadtxt(mapName, dtype=int, skiprows=1)
grid = np.loadtxt(gridName, dtype=int, skiprows=1)

x = map.shape[0]
y = map.shape[1]

colorMap = np.zeros((x, y, 3))
for i in range(x):
    for j in range(y):
        if grid[i][j] < -10:
            map[i][j] = (0, 0, 0) #black for obstacles/barriers
        elif map[i][j] == 0:
            colorMap[i][j] = (255, 0, 0) #red for wood
        elif map[i][j] == 1:
            colorMap[i][j] = (0, 255, 0) #green for wavy
        elif map[i][j] == 2:
            colorMap[i][j] = (0, 0, 255) #blue for hexagon
        elif map[i][j] == 3:
            colorMap[i][j] = (255, 255, 0) #yellow for carpet
        elif map[i][j] == 4:
            colorMap[i][j] = (128, 0, 128) #purple for blacktile
        elif map[i][j] == 5:
            colorMap[i][j] = (255, 255, 255) #white for unknown 
        
cmap = {1:[0, 0, 0], 2:[255, 0, 0], 3:[0, 255, 0], 4:[0, 0, 255], 5:[255, 255, 0], 6: [128, 0, 128], 7:[255, 255, 255]}
labels = {1:'Obstacle',2:'Wood',3:'Wavy', 4:'Hexagon',5:'Carpet',6:'Blacktile', 7:'Unknown'}

patches =[mpatches.Patch(color=cmap[i],label=labels[i]) for i in cmap]
plt.imshow(colorMap)
plt.legend(handles=patches, loc=4, borderaxespad=0.)
plt.show()