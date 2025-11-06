#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from numpy import arctan2
import sys, os
import simplekml
from pyproj import Proj
from GoBeaconNBack import mission, cvt_gll_ddmm_2_dd, PIDCapController, initialiser_fichier, enregistrer_valeurs,convert_cap_sensor_to_nav

# === Import des drivers DDBoat ===
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv
import gps_driver_v2 as gpddrv


# ==========================
# === PARAMÈTRES GLOBAUX ===
# ==========================

output_file = "log_mission_go_back.txt"
titres = "latitude\tlongitude\tdistance\tcap_act\tcap_obj\n"

offset_mot = -10
vitesse_base = 210

# Gains PID initiaux
Kp = 0.8
Ki = 0.03
Kd = 0.25

# Définition de la projection UTM zone 30 (France Ouest)
projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

# Points GPS
lat_ponton, lon_ponton = 48.19925, -3.01473


# ================================
# === CALIBRATION MAGNÉTOMÈTRE ===
# ================================

p = np.array([9.19344322e-08, 8.86431491e-08, 1.40532829e-08, 2.99160959e-08,
              -2.11090640e-08, 1.65789833e-08, 3.76555627e-05, -7.74120350e-05,
              -6.36399910e-05])

Q = np.array([[9.19344322e-08, 1.49580480e-08, -1.05545320e-08],
              [1.49580480e-08, 8.86431491e-08, 8.28949163e-09],
              [-1.05545320e-08, 8.28949163e-09, 1.40532829e-08]])

racineQ = np.array([[3.00888414e-04, 2.60587871e-05, -2.68613861e-05],
                    [2.60587871e-05, 2.95772384e-04, 2.19723862e-05],
                    [-2.68613861e-05, 2.19723862e-05, 1.13353267e-04]])

b = -0.5 * np.array([
    (1 / Q[0, 0]) * p[6],
    (1 / Q[1, 1]) * p[7],
    (1 / Q[2, 2]) * p[8]
])

# ======================
# === INITIALISATION ===
# ======================

initialiser_fichier(output_file, titres)
kml = simplekml.Kml()

ard = arddrv.ArduinoIO()
imu = imudrv.Imu9IO()
gps = gpddrv.GpsIO()
gps.set_filter_speed("0")

pid = PIDCapController(Kp, Ki, Kd, vitesse_base, offset_mot)

# =======================
# === LANCEMENT MISSION ===
# =======================

try:
    print("Démarrage de la mission DDBoat...\n")
    mission(lat_ponton, lon_ponton, label="Ponton")

finally:
    ard.send_arduino_cmd_motor(0, 0)
    kml.save("gps_data.kml")
    print("Données GPS enregistrées dans gps_data.kml")
    print("Mission terminée.\n")
