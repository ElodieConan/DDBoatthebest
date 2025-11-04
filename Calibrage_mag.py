
import matplotlib
matplotlib.use("TkAgg")
from roblib import *

p=  [ 2.76555971e-07 , 2.79324789e-07 , 2.30959703e-07 , 4.30784536e-08,
 -3.36328677e-08,  2.78418943e-08,  2.87369877e-04, -3.43639329e-04,
  1.27332450e-03]

Q=np.array([[ 2.76555971e-07,  2.15392268e-08, -1.68164339e-08],
 [ 2.15392268e-08 , 2.79324789e-07 , 1.39209471e-08],
 [-1.68164339e-08 , 1.39209471e-08 , 2.30959703e-07]])

racineQ = np.array( [[ 5.25203370e-04 , 2.06816012e-05 ,-1.70194899e-05],
 [ 2.06816012e-05,  5.27917190e-04,  1.41598059e-05],
 [-1.70194899e-05,  1.41598059e-05 , 4.80072432e-04]])

b = -0.5 * np.linalg.inv(Q) @ np.array([[p[6]],
                                        [p[7]],
                                        [p[8]]])
b=b.flatten()

long_deg = 64
long_rad = long_deg*np.pi/180
Z = np.array([[np.cos(long_rad), 0, -np.sin(long_rad)],
              [0, -np.cos(long_rad), 0],
              [-np.sin(long_rad), -np.sin(long_rad), np.cos(long_rad)]])

direction = ["north", "west", "up"]
Y = np.zeros((3,3))
for i in range(len(direction)):
    raw_data= np.loadtxt(f"mag_raw_data_{direction[i]}.txt", skiprows=1)
    n = raw_data.shape[0]
    data = np.zeros((n, 3))
    for k in range(n):
        data[k, :] = racineQ @ (raw_data[k, :] - b)
    x_mag, y_mag, z_mag = data[:,0], data[:,1], data[:,2]
    Y[:, i] = np.array([np.mean(x_mag), np.mean(y_mag), np.mean(z_mag)]).T
print(Y)
R_inu = Z@np.linalg.inv(Y)
print(R_inu)


