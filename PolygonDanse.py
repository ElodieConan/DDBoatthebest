#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import sys, os
import simplekml
from pyproj import Proj

# === Import des drivers DDBoat ===
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv
import gps_driver_v2 as gpddrv


# ==========================
# === PARAMÈTRES GLOBAUX ===
# ==========================

output_file = "log_mission_triangle.txt"
titres = "latitude\tlongitude\tdistance\tcap_act\tcap_obj\n"

offset_mot = -10
vitesse_base = 110

# Définition de la projection UTM zone 30 (France Ouest)
projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

# Points GPS
lat_ponton, lon_ponton = 48.19925, -3.01473
lat_a, lon_a   = 48.199269, -3.014768
lat_b, lon_b   = 48.199225, -3.015445
lat_c, lon_c   = 48.198852, -3.014891
lat_d, lon_d   = 48.198515, -3.015230
lat_e, lon_e   = 48.198698, -3.015790


# ======================
# === OUTILS FICHIERS ===
# ======================

def initialiser_fichier(file, titres):
    with open(file, 'w', encoding='utf-8') as f:
        f.write(titres)

def enregistrer_valeurs(file, lat, lon, dist, cap_act, cap_obj):
    line = "{}\t{}\t{}\t{}\t{}\n".format(lat, lon, dist, cap_act, cap_obj)
    try:
        with open(file, 'a', encoding='utf-8') as f:
            f.write(line)
    except Exception as e:
        print("Erreur écriture : {}".format(e))



# =======================
# === BIJECTION POLYGONE TRIGO ===
# =======================

def L(j,m):
    pass

# =======================
# === CONVERSIONS GPS ===
# =======================

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


# ==================================
# === CONVERSION CAP CAPTEUR/NAV ===
# ==================================

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi

def convert_cap_sensor_to_nav(cap_sensor):
    return (-cap_sensor + 180 - 10) % 360

def controller(cap_obj, cap_act):
    k1 = 15
    u1 = k1*sawtooth(cap_obj-cap_act)
    print("u1 = {}".format(u1))

    vit_mot_g = vitesse_base+u1-offset_mot
    vit_mot_d = vitesse_base-u1+offset_mot

    vit_mot_g = max(min(vit_mot_g, 255), 0)
    vit_mot_d = max(min(vit_mot_d, 255), 0)

    return vit_mot_g, vit_mot_d


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


# ============================
# === BOUCLE PRINCIPALE GPS ===
# ============================
reference_x, reference_y = projDegree2Meter(lon_ponton, lat_ponton)

def SuiviDeLigne(lat_a, lon_a, lat_b, lon_b, label):
    x_a, y_a = projDegree2Meter(lon_a, lat_a)
    pt_a = np.array([[x_a], [y_a]])
    x_b, y_b= projDegree2Meter(lon_b, lat_b)
    pt_b = np.array([[x_b], [y_b]])

    v_ab = pt_b-pt_a
    v_dir_uni_ab = v_ab/np.linalg.norm(v_ab)

    k3 = 5
    validation = 1
    while validation > 0:
        gll_ok, gll_data = gps.read_gll_non_blocking()
        if not gll_ok:
            time.sleep(0.05)
            continue

        lat, lon = cvt_gll_ddmm_2_dd(gll_data)
        x, y = projDegree2Meter(lon, lat)
        pt_m = np.array([[x], [y]])

        e = np.linalg.det(np.hstack((v_dir_uni_ab,pt_m-pt_a)))
        phi = np.arctan2(v_ab[1,0], v_ab[0,0])
        theta_bar = np.degrees(phi-np.tanh(e/k3))
        heading_geo = (90-theta_bar) % 360
        hdg_geo_rad = heading_geo*np.pi/180


        xmag, ymag, zmag = imu.read_mag_raw()
        Y = np.array([xmag, ymag, zmag])
        Y_corr = racineQ @ (Y - b)
        xmag_corr, ymag_corr, zmag_corr = Y_corr
        cap_act_rad = np.arctan2(ymag_corr, xmag_corr)
        cap_act = cap_act_rad* 180 / np.pi
        cap_nav = convert_cap_sensor_to_nav(cap_act)
        cap_nav_rad = cap_nav * np.pi/180
        print("cap_act = {}  cap_obj = {}".format(cap_nav, heading_geo))

        vg, vd = controller(hdg_geo_rad, cap_nav_rad)
        print("Moteurs: G={:.1f} D={:.1f}\n".format(vg, vd))

        ard.send_arduino_cmd_motor(vg, vd)
        enregistrer_valeurs(output_file, lat, lon, validation, cap_nav, heading_geo)
        kml.newpoint(name=label, coords=[(lon, lat)])

        validation = float(v_ab.T@(pt_b-pt_m))
        print("[{}] lat={:.5f} lon={:.5f} dist={:.2f} heading_geo={:.1f}".format(label, lat, lon, validation, heading_geo))
        time.sleep(0.1)

    print("Arrivé au point {} !".format(label))


# =======================
# === LANCEMENT MISSION ===
# =======================

try:
    print("Démarrage de la mission DDBoat...\n")
    SuiviDeLigne(lat_ponton, lon_ponton, lat_a, lon_a, label="Ponton->A")
    SuiviDeLigne(lat_a, lon_a, lat_b, lon_b, label="A->B")
    SuiviDeLigne(lat_b, lon_b, lat_c, lon_c, label="B->C")
    SuiviDeLigne(lat_c, lon_c, lat_a, lon_a, label="C->A")
    SuiviDeLigne(lat_a, lon_a, lat_ponton, lon_ponton, label="A->Ponton")

finally:
    ard.send_arduino_cmd_motor(0, 0)
    kml.save("gps_data_triangle.kml")
    print("Données GPS enregistrées dans gps_data_triangle.kml")
    print("Mission terminée.\n")
