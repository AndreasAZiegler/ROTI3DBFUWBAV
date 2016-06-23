#!/usr/bin/env python
'''
Created on 26.05.2016

@author: Andreas Ziegler
'''

import numpy as np
import h5py
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

file = h5py.File("./aruco_uwb_output.hdf5", "r")
aruco_coordinates = file['aruco_coordinates'][:]
uwb_coordinates = file['uwb_coordinates'][:]
file.close()

uwb = np.empty([len(uwb_coordinates), 4])
aruco = np.empty([len(uwb_coordinates), 4])

pos = 0
offset = 1
for i in range(len(uwb_coordinates)):
  for j in range(offset, len(aruco_coordinates)):
    # Value where aruco time > uwb time
    if uwb_coordinates[i,0] < aruco_coordinates[j,0]:
      # check the one before
      if abs(uwb_coordinates[i,0] - aruco_coordinates[j-1,0]) < 5:
        uwb[pos] = uwb_coordinates[i]
        aruco[pos] = aruco_coordinates[j-1]
        pos = pos + 1
        offset = j
        continue

# Remove empty rows
uwb = uwb[~(uwb==0).all(1)]
aruco = aruco[~(aruco==0).all(1)]

#print('Max. difference: {0}'.format(max(abs(uwb[:,0] - aruco[:,0]))))

# Calculate mean
mean_uwb = uwb[:,1:4].mean(0)
mean_aruco = aruco[:,1:4].mean(0)

"""
aruco_x = aruco[:,1]
aruco_y = -aruco[:,2]
aruco_z = aruco[:,3]
"""
aruco_x = aruco[:,1]
aruco_y = aruco[:,2]
aruco_z = aruco[:,3]

# Modify aruco data to check if rotation and translation are correct
"""
aruco_tmp = np.empty((164, 3))
aruco_tmp[:,0] = aruco_x
aruco_tmp[:,1] = aruco_y
aruco_tmp[:,2] = aruco_z
aruco_tmp = np.matmul((np.sqrt(2)/2)*np.array([[1, -1, 0], [1, 1, 0], [0, 0, 1]]), np.array(aruco_tmp.transpose()))
aruco_tmp = aruco_tmp + np.array([[100], [100], [100]])
aruco_x = aruco_tmp[0,:]
aruco_y = aruco_tmp[1,:]
aruco_z = aruco_tmp[2,:]
"""

# uwb_transf_1: 20160610_1736
"""
uwb_x = -uwb[:,2]
uwb_y = uwb[:,3]
uwb_z = uwb[:,1]
"""
uwb_x = uwb[:,1]
uwb_y = uwb[:,2]
uwb_z = uwb[:,3]

# uwb_1: video_uwb_1: lrms=0.1093
uwb_x1 = uwb_x + 0.1151
uwb_y1 = uwb_y - 0.0139
uwb_z1 = uwb_z + 0.0304

# uwb_2: video_uwb_2: lrms=0.1718
uwb_x2 = uwb_x + 0.0943
uwb_y2 = uwb_y - 0.0380
uwb_z2 = uwb_z - 0.0572

# uwb_3: video_uwb_3: lrms=0.1712
uwb_x3 = uwb_x + 0.0907
uwb_y3 = uwb_y - 0.0377
uwb_z3 = uwb_z - 0.0185

# uwb_4: video_uwb_4: lrms=0.1563
uwb_x4 = uwb_x + 0.0164
uwb_y4 = uwb_y + 0.0026
uwb_z4 = uwb_z - 0.1414

# uwb_5: video_uwb_5: lrms=0.1071
uwb_x5 = uwb_x + 0.0836
uwb_y5 = uwb_y - 0.0774
uwb_z5 = uwb_z - 0.2849

# uwb_6: video_uwb_6: lrms=0.0701
uwb_x6 = uwb_x - 0.0169
uwb_y6 = uwb_y - 0.0175
uwb_z6 = uwb_z - 0.2107


uwb_x = uwb_x6
uwb_y = uwb_y6
uwb_z = uwb_z6


# uwb_1: video_uwb_1: lrms=0.1093
uwb_transf_x1 = 0.9819*( 0.3072*uwb_x - 0.9216*uwb_y - 0.0086*uwb_z)
uwb_transf_y1 = 0.9819*( 0.1335*uwb_x + 0.0521*uwb_y - 0.9897*uwb_z)
uwb_transf_z1 = 0.9819*( 0.9422*uwb_x + 0.3029*uwb_y + 0.1430*uwb_z)

# uwb_2: video_uwb_2: lrms=0.1718
uwb_transf_x2 = 1.0135*( 0.3019*uwb_x - 0.9532*uwb_y + 0.0152*uwb_z)
uwb_transf_y2 = 1.0135*( 0.1027*uwb_x + 0.0166*uwb_y - 0.9946*uwb_z)
uwb_transf_z2 = 1.0135*( 0.9478*uwb_x + 0.3018*uwb_y + 0.1029*uwb_z)

# uwb_3: video_uwb_3: lrms=0.1712
uwb_transf_x3 = 1.0136*( 0.3184*uwb_x - 0.9388*uwb_y - 0.1317*uwb_z)
uwb_transf_y3 = 1.0136*( 0.0785*uwb_x + 0.1645*uwb_y - 0.9832*uwb_z)
uwb_transf_z3 = 1.0136*( 0.9447*uwb_x + 0.3027*uwb_y + 0.1261*uwb_z)

# uwb_4: video_uwb_4: lrms=0.1563  INVERTED Z-AXIS!!!!!
uwb_transf_x4 = 1.0002*(-0.1462*uwb_x - 0.9845*uwb_y + 0.0964*uwb_z)
uwb_transf_y4 = 1.0002*(-0.2051*uwb_x + 0.1255*uwb_y + 0.9707*uwb_z)
uwb_transf_z4 = 1.0002*(-0.9677*uwb_x + 0.1222*uwb_y - 0.2203*uwb_z)

# uwb_5: video_uwb_5: lrms=0.1071  INVERTED Z-AXIS!!!!!
uwb_transf_x5 = 1.0850*(-0.2063*uwb_x - 0.8790*uwb_y + 0.4299*uwb_z)
uwb_transf_y5 = 1.0850*(-0.3519*uwb_x + 0.4766*uwb_y + 0.8056*uwb_z)
uwb_transf_z5 = 1.0850*(-0.9130*uwb_x + 0.0149*uwb_y - 0.4077*uwb_z)

# uwb_6: video_uwb_6: lrms=0.0701
uwb_transf_x6 = 1.0355*( 0.1520*uwb_x - 0.9866*uwb_y + 0.0598*uwb_z)
uwb_transf_y6 = 1.0355*(-0.2253*uwb_x - 0.0935*uwb_y - 0.9698*uwb_z)
uwb_transf_z6 = 1.0355*( 0.9624*uwb_x + 0.1339*uwb_y - 0.2365*uwb_z)


uwb_transf_x = uwb_transf_x6
uwb_transf_y = uwb_transf_y6
uwb_transf_z = uwb_transf_z6


print('Aruco x difference: {0}m'.format(abs(max(aruco_x) - min(aruco_x))))
print('UWB x difference: {0}m'.format(abs(max(uwb_transf_x) - min(uwb_transf_x))))
print('Aruco y difference: {0}m'.format(abs(max(aruco_y) - min(aruco_y))))
print('UWB y difference: {0}m'.format(abs(max(uwb_transf_y) - min(uwb_transf_y))))
print('Aruco z difference: {0}m'.format(abs(max(aruco_z) - min(aruco_z))))
print('UWB z difference: {0}m'.format(abs(max(uwb_transf_z) - min(uwb_transf_z))))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#aruco_plt = ax.scatter(aruco_coordinates[:,1], aruco_coordinates[:,3], -aruco_coordinates[:,2], depthshade=True, c='g', marker='x', label='Aruco')
aruco_plt = ax.scatter(aruco_x, aruco_z, aruco_y, c='b', marker='x', label='Aruco')
uwb_transf_plt = ax.scatter(uwb_transf_x, uwb_transf_z, uwb_transf_y, c='r', marker='o', label='UWB Transformed')
plt.legend(handles=[uwb_transf_plt, aruco_plt])#, aruco_transf_plt, uwb_plt])
#plt.legend(handles=[uwb_plt])

ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')

plt.axis('equal')
plt.show()
