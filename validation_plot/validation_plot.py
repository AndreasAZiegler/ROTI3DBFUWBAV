#!/usr/bin/env python

# Imports
import h5py
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


# Load data sets
file = h5py.File("/data/SP1/catkin_ws/src/validation/validation_data.hdf5", "r")
uwb_coordinates = file['uwb'][:]
vicon_coordinates = file['vicon'][:]
ekf_coordinates = file['ekf'][:]
file.close()

# Remove delay of ekf
#for i in range(len(ekf_coordinates)):
#  ekf_coordinates[i,0] = ekf_coordinates[i,0] - 0 - (i * 8)

# Matching of the datasets according to their time stamps
vicon = np.zeros((len(vicon_coordinates), 4))
uwb = np.zeros((len(uwb_coordinates), 4))

pos = 0
offset = 1
break_flag = False
for i in range(len(vicon_coordinates)):
  for j in range(offset, len(uwb_coordinates)):
    # value where uwb time > vicon time
    if vicon_coordinates[i, 0] < uwb_coordinates[j, 0]:
      # check the one before
      if abs(vicon_coordinates[i, 0] - uwb_coordinates[j-1, 0]) < 5:
        vicon[pos] = vicon_coordinates[i]
        uwb[pos] = uwb_coordinates[j - 1]
        pos = pos + 1
        if pos >= len(uwb_coordinates):
          break_flag = True
          break
        offset = j
        continue
  if break_flag==True:
    break

# Remove empty rows
uwb = uwb[~(uwb==0).all(1)]
vicon = vicon[~(vicon==0).all(1)]

# uwb_30: video_uwb_30: lrms=0.0632
#uwb_x = uwb[:, 1] - 0.0297
#uwb_y = uwb[:, 2] + 0.0083
#uwb_z = uwb[:, 3] - 0.0568

# uwb_52: video_uwb_52: lrms=0.0579
uwb_x = uwb[:, 1] - 0.0673
uwb_y = uwb[:, 2] + 0.0129
uwb_z = uwb[:, 3] - 0.1133

"""
uwb_x = uwb_coordinates[:, 1] - 0.0297
uwb_y = uwb_coordinates[:, 2] + 0.0083
uwb_z = uwb_coordinates[:, 3] - 0.0568
"""

# uwb_30: video_uwb_30: lrms=0.0632
#uwb_x_wc = 1.0340*( 0.1203*uwb_x - 0.9910*uwb_y + 0.0595*uwb_z)
#uwb_y_wc = 1.0340*(-0.0356*uwb_x - 0.0642*uwb_y - 0.9973*uwb_z)
#uwb_z_wc = 1.0340*( 0.9921*uwb_x + 0.1178*uwb_y - 0.0430*uwb_z)

# uwb_52: video_uwb_52: lrms=0.0579
uwb_x_wc = 1.0429*( 0.0842*uwb_x - 0.9964*uwb_y + 0.0130*uwb_z)
uwb_y_wc = 1.0429*(-0.1456*uwb_x - 0.0252*uwb_y - 0.9890*uwb_z)
uwb_z_wc = 1.0429*( 0.9857*uwb_x + 0.0814*uwb_y - 0.1472*uwb_z)

# vicon_30: video_uwb_30: lrms=0.0336
#vicon_x = vicon[:, 1] - 1.4663
#vicon_y = vicon[:, 2] + 1.2641
#vicon_z = vicon[:, 3] - 1.2332

# vicon_40: video_uwb_40: lrms=0.0365
#vicon_x = vicon[:, 1] - 1.1373
#vicon_y = vicon[:, 2] + 0.9085
#vicon_z = vicon[:, 3] - 1.2501

# vicon_52: video_uwb_52: lrms=0.0376
vicon_x = vicon[:, 1] - 1.8605
vicon_y = vicon[:, 2] + 0.8850
vicon_z = vicon[:, 3] - 1.2545

# vicon_30: video_uwb_30: lrms=0.0336
#vicon_x_wc = 1.0662*(-0.0350*vicon_x + 0.9993*vicon_y - 0.0113*vicon_z)
#vicon_y_wc = 1.0662*( 0.0578*vicon_x - 0.0092*vicon_y - 0.9983*vicon_z)
#vicon_z_wc = 1.0662*(-0.9977*vicon_x - 0.0356*vicon_y - 0.0575*vicon_z)

# vicon_40: video_uwb_40: lrms=0.0365
#vicon_x_wc = 1.0444*( 0.0207*vicon_x + 0.9996*vicon_y - 0.0202*vicon_z)
#vicon_y_wc = 1.0444*( 0.0281*vicon_x - 0.0208*vicon_y - 0.9994*vicon_z)
#vicon_z_wc = 1.0444*(-0.9994*vicon_x + 0.0202*vicon_y - 0.0285*vicon_z)

# vicon_52: video_uwb_52: lrms=0.0376
vicon_x_wc = 1.0529*(-0.0758*vicon_x + 0.9969*vicon_y - 0.0200*vicon_z)
vicon_y_wc = 1.0529*( 0.0245*vicon_x - 0.0182*vicon_y - 0.9995*vicon_z)
vicon_z_wc = 1.0529*(-0.9968*vicon_x + 0.0762*vicon_y - 0.0230*vicon_z)

# Calculate rmse
var_uwb = []
var_uwb_xy = []
for i in range(len(uwb_x_wc)):
  #rmse_uwb[i] = np.sqrt((vicon[1,i] - uwb[1,i])**2 + (vicon[2,i] - uwb[2,i])**2 + (vicon[3,i] - uwb[3,i])**2)
  var_uwb.append(((vicon_x_wc[i] - uwb_x_wc[i])**2 + (vicon_y_wc[i] - uwb_y_wc[i])**2 + (vicon_z_wc[i] - uwb_z_wc[i])**2))
  var_uwb_xy.append(((vicon_x_wc[i] - uwb_x_wc[i])**2 + (vicon_y_wc[i] - uwb_y_wc[i])**2))

print("UWB rmse = {0}".format(np.sqrt(1/(len(var_uwb) - 1) * sum(var_uwb))))
print("UWB rmse_xy = {0}".format(np.sqrt(1/(len(var_uwb_xy) - 1) * sum(var_uwb_xy))))

# Plot variance of uwb
plt.figure(1)
plt.plot(uwb[:, 0], var_uwb)


# Matching of the datasets according to their time stamps
vicon = np.zeros((len(vicon_coordinates), 4))
ekf = np.zeros((len(ekf_coordinates), 4))

pos = 0
offset = 1
break_flag = False
for i in range(len(vicon_coordinates)):
  for j in range(offset, len(ekf_coordinates)):
    # value where ekf time > vicon time
    if vicon_coordinates[i, 0] < ekf_coordinates[j, 0]:
      # check the one before
      if abs(vicon_coordinates[i, 0] - ekf_coordinates[j-1, 0]) < 5:
        vicon[pos] = vicon_coordinates[i]
        ekf[pos] = ekf_coordinates[j - 1]
        pos = pos + 1
        if pos >= len(ekf_coordinates):
          break_flag = True
          break
        offset = j
        continue
  if break_flag==True:
    break

# Remove empty rows
ekf = ekf[~(ekf==0).all(1)]
vicon = vicon[~(vicon==0).all(1)]

# vicon_30: video_uwb_30: lrms=0.0336
#vicon_x = vicon[:, 1] - 1.4663
#vicon_y = vicon[:, 2] + 1.2641
#vicon_z = vicon[:, 3] - 1.2332

# vicon_40: video_uwb_40: lrms=0.0365
#vicon_x = vicon[:, 1] - 1.1373
#vicon_y = vicon[:, 2] + 0.9085
#vicon_z = vicon[:, 3] - 1.2501

# vicon_52: video_uwb_52: lrms=0.0376
vicon_x = vicon[:, 1] - 1.8605
vicon_y = vicon[:, 2] + 0.8850
vicon_z = vicon[:, 3] - 1.2545

"""
vicon_x = vicon_coordinates[:, 1] - 1.4663
vicon_y = vicon_coordinates[:, 2] + 1.2641
vicon_z = vicon_coordinates[:, 3] - 1.2332
"""

# vicon_30: video_uwb_30: lrms=0.0336
#vicon_x_wc = 1.0662*(-0.0350*vicon_x + 0.9993*vicon_y - 0.0113*vicon_z)
#vicon_y_wc = 1.0662*( 0.0578*vicon_x - 0.0092*vicon_y - 0.9983*vicon_z)
#vicon_z_wc = 1.0662*(-0.9977*vicon_x - 0.0356*vicon_y - 0.0575*vicon_z)

# vicon_40: video_uwb_40: lrms=0.0365
#vicon_x_wc = 1.0444*( 0.0207*vicon_x + 0.9996*vicon_y - 0.0202*vicon_z)
#vicon_y_wc = 1.0444*( 0.0281*vicon_x - 0.0208*vicon_y - 0.9994*vicon_z)
#vicon_z_wc = 1.0444*(-0.9994*vicon_x + 0.0202*vicon_y - 0.0285*vicon_z)

# vicon_52: video_uwb_52: lrms=0.0376
vicon_x_wc = 1.0529*(-0.0758*vicon_x + 0.9969*vicon_y - 0.0200*vicon_z)
vicon_y_wc = 1.0529*( 0.0245*vicon_x - 0.0182*vicon_y - 0.9995*vicon_z)
vicon_z_wc = 1.0529*(-0.9968*vicon_x + 0.0762*vicon_y - 0.0230*vicon_z)

# Calculate rmse
var_ekf = []
var_ekf_xy = []

for i in range(len(ekf)):
  #rmse_ekf[i] = np.sqrt((vicon[1,i] - ekf[1,i])**2 + (vicon[2,i] - ekf[2,i])**2 + (vicon[3,i] - ekf[3,i])**2)
  var_ekf.append(((vicon_x_wc[i] - ekf[i, 1])**2 + (vicon_y_wc[i] - ekf[i, 2])**2 + (vicon_z_wc[i] - ekf[i, 3])**2))
  var_ekf_xy.append(((vicon_x_wc[i] - ekf[i, 1])**2 + (vicon_y_wc[i] - ekf[i, 2])**2))

print("EKF rmse = {0}".format(np.sqrt(1/(len(var_ekf) - 1) * sum(var_ekf))))
print("EKF rmse_xy = {0}".format(np.sqrt(1/(len(var_ekf_xy) - 1) * sum(var_ekf_xy))))

# Plot variance of ekf
plt.figure(2)
plt.plot(ekf[:, 0], var_ekf)

fig = plt.figure(3)
ax = fig.add_subplot(111, projection='3d')

# 3D plot of vicon, uwb and ekf
vicon_plt = ax.plot(vicon_x_wc, vicon_z_wc, -vicon_y_wc, c='magenta', label='VICON')
#vicon_plt = ax.plot(vicon_coordinates[:,1], vicon_coordinates[:,3], -vicon_coordinates[:,2], c='b', label='VICON')
uwb_plt = ax.plot(uwb_x_wc, uwb_z_wc, -uwb_y_wc, c='r', label='UWB')
ekf_plt = ax.plot(ekf[:, 1], ekf[:, 3], -ekf[:, 2], c='green', label='EKF')
##ekf_plt_orig = ax.plot(ekf_coordinates[:, 1], ekf_coordinates[:, 3], -ekf_coordinates[:, 2], c='y', label='EKF orig')
plt.legend()
ax.set_xlabel('X (image plane) [m]')
ax.set_ylabel('Z (distance) [m]')
ax.set_zlabel('Y (image plane) [m]')

plt.axis('equal')
plt.show()
print("End")

