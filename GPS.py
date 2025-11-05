import sys
import os
import time
import numpy as np

# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import gps_driver_v2 as gpddrv

import simplekml
# Create aKML object
kml = simplekml.Kml()

from pyproj import Proj, transform
projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")



def cvt_gll_ddmm_2_dd(st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat / 100))
    olon = float(int(ilon / 100))
    olat_mm = (ilat % 100) / 60
    olon_mm = (ilon % 100) / 60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat, olon

#reference du bout du ponton
lat_ponton =48.198238
lon_ponton =-3.015003
altitude=121       #en mètres
lat_bouee=48.19932
lon_bouee=-3.01320
reference_x,reference_y= projDegree2Meter(lon_ponton,lat_ponton)

gps = gpddrv.GpsIO()  # create a GPS object
gps.set_filter_speed("0")  # allowing GPS measures to change even if the DDBoat is not moving
cnt = 150  # takes 5 GPS measures
while True:
    gll_ok, gll_data = gps.read_gll_non_blocking()

    if gll_ok:  # GPGLL message received
        lat, lon = cvt_gll_ddmm_2_dd(gll_data)  # convert DDMM.MMMM toDD.DDDDD
        x, y = projDegree2Meter(lon, lat)  # convert to meters
        lat_check, lon_check = projDegree2Meter(x, y, inverse=True)  # check conversion OK
        dx = x - reference_x
        dy = y - reference_y
        distance = np.sqrt(dx * dx + dy * dy)
        heading_trigo = np.degrees(np.arctan2(dy, dx))
        heading_geo = 90.0 - heading_trigo  # convert from trigonomety togeographic
        print("lat=%.4flon=%.4f(check %.4f %.4f)x=%.2fy=%.2fdx=%.2f, dy=%.2f, distance=%.2f, heading=%.2f" %
              (lat, lon, lat_check, lon_check, x, y, dx, dy, distance, heading_geo))
        pnt = kml.newpoint(name="GPS", coords=[(lon, lat)])
        cnt -= 1
        if cnt == 0:
            break
    time.sleep(0.01)

kml.save("gps_data.kml")