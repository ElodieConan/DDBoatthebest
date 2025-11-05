
import time
import numpy as np
from numpy.ma.core import arctan2

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

output_file = "log_mission_bouee_05_11_2025.txt"
titres = "latitude\tlongitude\tdistance au waypoint\tcap\tcap à suivre\n"

def initialiser_fichier(file, titres):
    with open(file, 'w') as f:
        f.write("titres\n")

initialiser_fichier(output_file, titres)



# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv

offset_mot = -10
k = 0.5  # gain de correction
ard = arddrv.ArduinoIO()  # create an ARduino object
imu = imudrv.Imu9IO()
# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import gps_driver_v2 as gpddrv

import simplekml

# Create aKML object
kml = simplekml.Kml()

from pyproj import Proj, transform

projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

def enregistrer_valeurs(file, lat, long, dist, cap, cap_com):
    line = "{} {} {} {} {}\n".format(lat, long, dist, cap, cap_com)
    try:
        with open(file, 'a', encoding='utf-8') as f:
            f.write(line)
    except Exception as e:
        print("Erreur lors de l'écriture : {}".format(e))

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


# On garde ta fonction de conversion telle qu’elle :
def convert_cap_sensor_to_nav(cap_sensor):
    """Convertit le cap du capteur vers le système NAV (N=0°, E=90°, S=180°, O=270°)."""
    return (-cap_sensor + 180- 10) % 360


class PIDCapController:
    def __init__(self, Kp=1.2, Ki=0.02, Kd=0.6, vitesse_base=110, offset_mot=0):
        """
        PID simple pour le contrôle de cap du DDBoat.
        Gains à ajuster selon comportement réel.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.vitesse_base = vitesse_base
        self.offset_mot = offset_mot

        self.integrale = 0.0
        self.erreur_prec = 0.0
        self.t_prec = time.time()

    def correction_cap(self, cap_obj, cap_act_sensor):
        """Retourne vit_mot_g, vit_mot_d bornées dans [0, 255]."""

        # --- Conversion du cap capteur en cap NAV ---
        cap_act = convert_cap_sensor_to_nav(cap_act_sensor)

        # --- Calcul erreur de cap dans [-180, 180] ---
        erreur = cap_obj - cap_act
        if erreur > 180:
            erreur -= 360
        elif erreur < -180:
            erreur += 360

        # --- PID ---
        t = time.time()
        dt = t - self.t_prec if self.t_prec else 0.01
        self.t_prec = t

        # Intégrale (limitée pour éviter les débordements)
        self.integrale += erreur * dt
        self.integrale = max(min(self.integrale, 1000), -1000)

        # Dérivée
        derivee = (erreur - self.erreur_prec) / dt if dt > 0 else 0.0
        self.erreur_prec = erreur

        # Commande PID
        correction = self.Kp * erreur + self.Ki * self.integrale + self.Kd * derivee

        # --- Application à la commande moteur ---
        vit_mot_g = self.vitesse_base + correction - self.offset_mot
        vit_mot_d = self.vitesse_base - correction + self.offset_mot

        # Saturation dans [0, 255]
        vit_mot_g = max(min(vit_mot_g, 255), 0)
        vit_mot_d = max(min(vit_mot_d, 255), 0)

        return vit_mot_g, vit_mot_d


def angles_Euler(a1, y1):
    r1 = y1 - ((y1.T @ a1) * a1)
    normr1 = np.linalg.norm(r1)
    v = (y1 - (y1.T @ a1) * a1).flatten()
    r2 = np.cross(a1.flatten(), v)
    normr2 = np.linalg.norm(r2)
    R = np.column_stack((r1 / normr1, r2 / normr2, a1))
    phi = np.arctan2(R[2, 1], R[2, 2])
    theta = -np.arcsin(R[2, 0])
    psi = np.arctan2(R[1, 0], R[0, 0])
    return phi, theta, psi


# transformation des données mag en cap
# Données de correction :

p = np.array([9.19344322e-08, 8.86431491e-08, 1.40532829e-08, 2.99160959e-08,
              -2.11090640e-08, 1.65789833e-08, 3.76555627e-05, -7.74120350e-05,
              -6.36399910e-05])
racineQ = np.array([[3.00888414e-04, 2.60587871e-05, -2.68613861e-05],
                    [2.60587871e-05, 2.95772384e-04, 2.19723862e-05],
                    [-2.68613861e-05, 2.19723862e-05, 1.13353267e-04]])

Q = np.array([[9.19344322e-08, 1.49580480e-08, -1.05545320e-08],
              [1.49580480e-08, 8.86431491e-08, 8.28949163e-09],
              [-1.05545320e-08, 8.28949163e-09, 1.40532829e-08]])

# --- Calcul du vecteur b ---
b = -0.5 * np.array([
    (1 / Q[0, 0]) * p[6],
    (1 / Q[1, 1]) * p[7],
    (1 / Q[2, 2]) * p[8]
])

a0 = np.array([[0], [0], [1]])

a1 = a0  # hypothèse si il n'y a pas de vagues, à améliorer pour après
pid = PIDCapController(Kp=1.5, Ki=0.03, Kd=0.5, vitesse_base=110)

# reference du bout du ponton
lat_ponton = 48.198238
lon_ponton = -3.015003
altitude = 121  # en mètres
lat_bouee = 48.19932
lon_bouee = -3.01320
reference_x, reference_y = projDegree2Meter(lon_bouee, lat_bouee)

gps = gpddrv.GpsIO()  # create a GPS object
gps.set_filter_speed("0")  # allowing GPS measures to change even if the DDBoat is not moving
cnt = 50  # takes 5 GPS measures
distance = 40

while distance > 0.5:
    gll_ok, gll_data = gps.read_gll_non_blocking()

    if gll_ok:  # GPGLL message received
        lat, lon = cvt_gll_ddmm_2_dd(gll_data)  # convert DDMM.MMMM toDD.DDDDD
        x, y = projDegree2Meter(lon, lat)  # convert to meters
        lon_check, lat_check = projDegree2Meter(x, y, inverse=True)  # check conversion OK
        dx = x - reference_x
        dy = y - reference_y
        distance = np.sqrt(dx * dx + dy * dy)
        heading_trigo = np.degrees(np.arctan2(dy, dx))
        heading_geo = (heading_trigo+15)%360  # convert from trigonomety togeographic
        print("lat=%.4flon=%.4f(check %.4f %.4f)x=%.2fy=%.2fdx=%.2f, dy=%.2f, distance=%.2f, heading=%.2f" %
              (lat, lon, lat_check, lon_check, x, y, dx, dy, distance, heading_geo))
        pnt = kml.newpoint(name="GPS", coords=[(lon, lat)])
        cap_obj = heading_geo
        # --- Lecture du magnétomètre ---
        xmag, ymag, zmag = imu.read_mag_raw()

        # --- Mise sous forme de vecteur colonne ---
        Y = np.array([xmag, ymag, zmag])
        Y_corr = racineQ @ (Y - b)

        # --- Résultats corrigés ---
        xmag_corr, ymag_corr, zmag_corr = Y_corr
        Y1 = Y_corr.reshape(3, 1)  # même format que ton ancienne version
        # acceleration
        # R_inu = np.array([[-0.5544297, 0.12721476, 0.00438503],
        #                  [0.07167656, 0.59082185, -0.10687039],
        #                  [0.45592243, 0.32278327, -0.53408991]])

        # Y1 = R_inu @ Y1

        # cap_act=angles_Euler(a1,Y1)[2]*180/np.pi
        cap_act = arctan2(ymag_corr, xmag_corr) * 180 / np.pi
        print("cap after rot:", convert_cap_sensor_to_nav(cap_act))
        left_speed, right_speed = pid.correction_cap(cap_obj, cap_act)
        print(left_speed, right_speed)
        ard.send_arduino_cmd_motor(left_speed, right_speed)  # in place turn
        enregistrer_valeurs(output_file,lat, lon, distance, convert_cap_sensor_to_nav(cap_act), heading_geo)
        cnt -= 1
        if cnt<0: break
    time.sleep(0.01)
distance = 40
reference_x, reference_y = projDegree2Meter(lon_ponton, lat_ponton)
cnt = 50
while distance > 0.5:
    gll_ok, gll_data = gps.read_gll_non_blocking()
    if gll_ok:  # GPGLL message received
        lat, lon = cvt_gll_ddmm_2_dd(gll_data)  # convert DDMM.MMMM toDD.DDDDD
        x, y = projDegree2Meter(lon, lat)  # convert to meters
        lon_check, lat_check = projDegree2Meter(x, y, inverse=True)  # check conversion OK
        dx = x - reference_x
        dy = y - reference_y
        distance = np.sqrt(dx * dx + dy * dy)
        heading_trigo = np.degrees(np.arctan2(dy, dx))
        heading_geo = (heading_trigo+15)%360  # convert from trigonomety togeographic
        print("lat=%.4flon=%.4f(check %.4f %.4f)x=%.2fy=%.2fdx=%.2f, dy=%.2f, distance=%.2f, heading=%.2f" %
              (lat, lon, lat_check, lon_check, x, y, dx, dy, distance, heading_geo))
        pnt = kml.newpoint(name="GPS", coords=[(lon, lat)])
        cap_obj = heading_geo
        # --- Lecture du magnétomètre ---
        xmag, ymag, zmag = imu.read_mag_raw()

        # --- Mise sous forme de vecteur colonne ---
        Y = np.array([xmag, ymag, zmag])
        Y_corr = racineQ @ (Y - b)

        # --- Résultats corrigés ---
        xmag_corr, ymag_corr, zmag_corr = Y_corr
        Y1 = Y_corr.reshape(3, 1)  # même format que ton ancienne version
        # acceleration
        # R_inu = np.array([[-0.5544297, 0.12721476, 0.00438503],
        #                  [0.07167656, 0.59082185, -0.10687039],
        #                  [0.45592243, 0.32278327, -0.53408991]])

        # Y1 = R_inu @ Y1

        # cap_act=angles_Euler(a1,Y1)[2]*180/np.pi
        cap_act = arctan2(ymag_corr, xmag_corr) * 180 / np.pi
        print("cap after rot:", convert_cap_sensor_to_nav(cap_act))
        left_speed, right_speed = pid.correction_cap(cap_obj, cap_act)
        print(left_speed, right_speed)
        ard.send_arduino_cmd_motor(left_speed, right_speed)  # in place turn
        enregistrer_valeurs(output_file, lat, lon, distance, convert_cap_sensor_to_nav(cap_act), heading_geo)
    time.sleep(0.01)
    cnt -= 1
    if cnt < 0: break

kml.save("gps_data.kml")

print("\n Programme fini\n")