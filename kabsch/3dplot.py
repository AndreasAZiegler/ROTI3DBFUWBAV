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

# uwb_7: 20160620_2347
uwb_x7 = uwb_x - 0.8244
uwb_y7 = uwb_y + 0.1260
uwb_z7 = uwb_z + 0.1454

# uwb_8: 20160621_0731
uwb_x8 = uwb_x - 0.4582
uwb_y8 = uwb_y - 0.5466
uwb_z8 = uwb_z + 0.3030

uwb_x = uwb_x8
uwb_y = uwb_y8
uwb_z = uwb_z8


# uwb_transf_7: 20160620_2347
uwb_transf_x7 = 1.1361*( 0.0159*uwb_x + 0.3724*uwb_y + 0.9279*uwb_z)
uwb_transf_y7 = 1.1361*(-0.0141*uwb_x + 0.9280*uwb_y - 0.3722*uwb_z)
uwb_transf_z7 = 1.1361*(-0.9998*uwb_x - 0.0072*uwb_y + 0.0200*uwb_z)

# uwb_transf_8: 20160621_0731
uwb_transf_x8 = 1.1672*( 0.6970*uwb_x - 0.7082*uwb_y + 0.1124*uwb_z)
uwb_transf_y8 = 1.1672*( 0.1812*uwb_x + 0.3257*uwb_y + 0.9280*uwb_z)
uwb_transf_z8 = 1.1672*(-0.6938*uwb_x - 0.9264*uwb_y + 0.3553*uwb_z)

uwb_transf_x = uwb_transf_x8
uwb_transf_y = uwb_transf_y8
uwb_transf_z = uwb_transf_z8


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
