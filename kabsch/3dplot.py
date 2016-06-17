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

aruco_x = aruco[:,1]
aruco_y = -aruco[:,2]
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
uwb_x = -uwb[:,2]
uwb_y = uwb[:,3]
uwb_z = uwb[:,1]

# uwb_1: 20160610_1736
uwb_x1 = uwb_x - 0.0230
uwb_y1 = uwb_y - 0.0421
uwb_z1 = uwb_z - 0.0063
# Modified
#uwb_x = uwb_x + 189.8489
#uwb_y = uwb_y - 14.1561
#uwb_z = uwb_z + 146.5468
# Modified transpose
#uwb_x = uwb_x - 189.8489
#uwb_y = uwb_y + 14.1561
#uwb_z = uwb_z - 146.5468

# uwb_2: 20160610_1738
uwb_x2 = uwb_x + 0.0007
uwb_y2 = uwb_y - 0.0832
uwb_z2 = uwb_z - 0.0496

# uwb_3: 20160610_1739
uwb_x3 = uwb_x - 0.0048
uwb_y3 = uwb_y - 0.0761
uwb_z3 = uwb_z - 0.0894

# uwb_4: 20160610_1805
uwb_x4 = uwb_x + 0.0570
uwb_y4 = uwb_y + 0.0388
uwb_z4 = uwb_z + 0.0359

# uwb_5: 20160610_1806
uwb_x5 = uwb_x + 0.0595
uwb_y5 = uwb_y + 0.0531
uwb_z5 = uwb_z + 0.0189

uwb_x = uwb_x4
uwb_y = uwb_y4
uwb_z = uwb_z4

# uwb_transf_1: 20160610_1736
uwb_transf_x1 = 0.9584*( 0.9924*uwb_x + 0.0142*uwb_y + 0.1219*uwb_z)
uwb_transf_y1 = 0.9584*(-0.0314*uwb_x + 0.9896*uwb_y + 0.1404*uwb_z)
uwb_transf_z1 = 0.9584*(-0.1187*uwb_x - 0.1431*uwb_y + 0.9826*uwb_z)
# Modified
#uwb_transf_x1 = 0.7209*( 0.7270*uwb_x - 0.6840*uwb_y - 0.0609*uwb_z)
#uwb_transf_y1 = 0.7209*( 0.6853*uwb_x + 0.7169*uwb_y + 0.1280*uwb_z)
#uwb_transf_z1 = 0.7209*(-0.0439*uwb_x - 0.1348*uwb_y + 0.9899*uwb_z)
# Modified transpose
#uwb_transf_x1 = 0.7209*( 0.7270*uwb_x + 0.6853*uwb_y - 0.0439*uwb_z)
#uwb_transf_y1 = 0.7209*(-0.6840*uwb_x + 0.7169*uwb_y - 0.1348*uwb_z)
#uwb_transf_z1 = 0.7209*(-0.0609*uwb_x + 0.1280*uwb_y + 0.9899*uwb_z)

# uwb_transf_2: 20160610_1738
uwb_transf_x2 = 0.9890*( 0.9960*uwb_x - 0.0111*uwb_y + 0.0892*uwb_z)
uwb_transf_y2 = 0.9890*(-0.0039*uwb_x + 0.9860*uwb_y + 0.1665*uwb_z)
uwb_transf_z2 = 0.9890*(-0.0898*uwb_x - 0.1661*uwb_y + 0.9820*uwb_z)

# uwb_transf_3: 20160610_1739
uwb_transf_x3 = 1.0382*( 0.9920*uwb_x - 0.0981*uwb_y + 0.0795*uwb_z)
uwb_transf_y3 = 1.0382*( 0.0825*uwb_x + 0.9802*uwb_y + 0.1801*uwb_z)
uwb_transf_z3 = 1.0382*(-0.0956*uwb_x - 0.1721*uwb_y + 0.9804*uwb_z)

# uwb_transf_4: 20160610_1805
uwb_transf_x4 = 0.9834*( 0.9956*uwb_x + 0.0861*uwb_y + 0.0380*uwb_z)
uwb_transf_y4 = 0.9834*(-0.0885*uwb_x + 0.9938*uwb_y + 0.0674*uwb_z)
uwb_transf_z4 = 0.9834*(-0.0320*uwb_x - 0.0704*uwb_y + 0.9970*uwb_z)

# uwb_transf_5: 20160610_1806
uwb_transf_x5 = 0.9883*( 0.9974*uwb_x + 0.0459*uwb_y + 0.0558*uwb_z)
uwb_transf_y5 = 0.9883*(-0.0482*uwb_x + 0.9981*uwb_y + 0.0396*uwb_z)
uwb_transf_z5 = 0.9883*(-0.0539*uwb_x - 0.0421*uwb_y + 0.9977*uwb_z)

uwb_transf_x = uwb_transf_x4
uwb_transf_y = uwb_transf_y4
uwb_transf_z = uwb_transf_z4

"""
# aruco_transf_1: 20160610_1736
aruco_x1 = 1.0434*aruco[:,1]
aruco_y1 = -1.0434*aruco[:,2]
aruco_z1 = 1.0434*aruco[:,3]
# aruco_transf_2: 20160610_1738
aruco_x2 = 1.0111*aruco[:,1]
aruco_y2 = -1.0111*aruco[:,2]
aruco_z2 = 1.0111*aruco[:,3]
# aruco_transf_3: 20160610_1739
aruco_x3 = 0.9632*aruco[:,1]
aruco_y3 = -0.9632*aruco[:,2]
aruco_z3 = 0.9632*aruco[:,3]
# aruco_transf_4: 20160610_1805
aruco_x4 = 1.0169*aruco[:,1]
aruco_y4 = -1.0169*aruco[:,2]
aruco_z4 = 1.0169*aruco[:,3]
# aruco_transf_5: 20160610_1806
aruco_x5 = 1.0119*aruco[:,1]
aruco_y5 = -1.0119*aruco[:,2]
aruco_z5 = 1.0119*aruco[:,3]

aruco_x = aruco_x2
aruco_y = aruco_y2
aruco_z = aruco_z2

# aruco_transf_1: 20160610_1736
aruco_transf_x1 = 0.9924*aruco_x - 0.0314*aruco_y - 0.1187*aruco_z + 0.0230
aruco_transf_y1 = 0.0142*aruco_x + 0.9996*aruco_y - 0.1431*aruco_z + 0.0421
aruco_transf_z1 = 0.1219*aruco_x + 0.1404*aruco_y + 0.9826*aruco_z + 0.0063
# aruco_transf_2: 20160610_1738
aruco_transf_x2 = 0.9960*aruco_x - 0.0039*aruco_y - 0.0898*aruco_z - 7.4764*10**(-4)
aruco_transf_y2 = -0.0111*aruco_x + 0.9860*aruco_y - 0.1661*aruco_z + 0.0832
aruco_transf_z2 = 0.0892*aruco_x + 0.1665*aruco_y + 0.9820*aruco_z + 0.0496
# aruco_transf_3: 20160610_1739
aruco_transf_x3 = 0.9920*aruco_x + 0.0825*aruco_y - 0.0956*aruco_z + 0.0048
aruco_transf_y3 = -0.0981*aruco_x + 0.9802*aruco_y - 0.1721*aruco_z + 0.0761
aruco_transf_z3 = 0.0795*aruco_x + 0.1801*aruco_y + 0.9804*aruco_z + 0.0894
# aruco_transf_4: 20160610_1805
aruco_transf_x4 = 0.9956*aruco_x - 0.0885*aruco_y - 0.0320*aruco_z - 0.0570
aruco_transf_y4 = 0.0861*aruco_x + 0.9938*aruco_y - 0.0704*aruco_z - 0.0388
aruco_transf_z4 = 0.0380*aruco_x + 0.0674*aruco_y + 0.9970*aruco_z - 0.0359
# aruco_transf_5: 20160610_1806
aruco_transf_x5 = 0.9974*aruco_x - 0.0482*aruco_y - 0.0539*aruco_z - 0.0595
aruco_transf_y5 = 0.0459*aruco_x + 0.9981*aruco_y - 0.0421*aruco_z - 0.0531
aruco_transf_z5 = 0.0558*aruco_x + 0.0396*aruco_y + 0.9977*aruco_z - 0.0189

aruco_transf_x = aruco_transf_x1
aruco_transf_y = aruco_transf_y1
aruco_transf_z = aruco_transf_z1
"""

# Print x and y max differences
#print('Aruco x difference: {0}m'.format(abs(max(aruco_coordinates[:,1]) - min(aruco_coordinates[:,1]))))
#print('UWB x difference: {0}m'.format(abs(max(uwb_coordinates[:,1]) - min(uwb_coordinates[:,1]))))
#print('Aruco y difference: {0}m'.format(abs(max(aruco_coordinates[:,2]) - min(aruco_coordinates[:,2]))))
#print('UWB y difference: {0}m'.format(abs(max(uwb_coordinates[:,2]) - min(uwb_coordinates[:,2]))))
#print('Aruco z difference: {0}m'.format(abs(max(aruco_coordinates[:,3]) - min(aruco_coordinates[:,3]))))
#print('UWB z difference: {0}m'.format(abs(max(uwb_coordinates[:,3]) - min(uwb_coordinates[:,3]))))

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
