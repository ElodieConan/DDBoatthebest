
import numpy as np
from pyproj import Proj
projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

lat_a, lon_a   = 48.199269, -3.014768
lat_b, lon_b   = 48.199225, -3.015445
lat_c, lon_c   = 48.198852, -3.014891
lat_d, lon_d   = 48.198515, -3.015230
lat_e, lon_e   = 48.198698, -3.015790

points_GPS = np.array([[lat_a, lon_a],
                       [lat_b, lon_b],
                       [lat_c, lon_c],
                       [lat_d, lon_d],
                       [lat_e, lon_e]]).T

N = np.shape(points_GPS)[1]

points_m = np.zeros((2, N))

for i in range(N):
    x, y = projDegree2Meter(points_GPS[1,i], points_GPS[0,i])
    points_m[0,i] = x
    points_m[1,i] = y
    #np.hstack((points_m, np.array([[x], [y]])))

distance = 0

for i in range(N):
    print("saluit")
    segment = points_m[:,(i+1)%N]-points_m[:,i]
    print(segment)
    distance += np.sqrt(segment[0]**2+segment[1]**2)
    #distance += np.linalg.norm(segment)

a = distance/(2*np.pi)
print("Coefficient de la bijection:", a)
print(distance)