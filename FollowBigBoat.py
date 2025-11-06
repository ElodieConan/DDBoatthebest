#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from numpy import arctan2
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

output_file = "log_mission_bouee.txt"
titres = "latitude\tlongitude\tdistance\tcap_act\tcap_obj\n"

offset_mot = -10
vitesse_base = 210

# Gains PID initiaux
Kp = 0.8
Ki = 0.03
Kd = 0.2

# Définition de la projection UTM zone 30 (France Ouest)
projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

# Points GPS
lat_ponton, lon_ponton = 48.19924, -3.01479
lat_bouee, lon_bouee   = 48.19923, -3.01617


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

def convert_cap_sensor_to_nav(cap_sensor):
    return (-cap_sensor + 180 - 10) % 360


# ==========================
# === CONTRÔLEUR PID CAP ===
# ==========================

class PIDCapController:
    def __init__(self, Kp, Ki, Kd, vitesse_base=210, offset_mot=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.vitesse_base = vitesse_base
        self.offset_mot = offset_mot
        self.integrale = 0.0
        self.erreur_prec = 0.0
        self.t_prec = time.time()

    def correction_cap(self, cap_obj, cap_act_sensor):
        cap_act = convert_cap_sensor_to_nav(cap_act_sensor)

        erreur = cap_obj - cap_act
        if erreur > 180:
            erreur -= 360
        elif erreur < -180:
            erreur += 360

        t = time.time()
        dt = t - self.t_prec if self.t_prec else 0.01
        self.t_prec = t

        self.integrale += erreur * dt
        self.integrale = max(min(self.integrale, 1000), -1000)

        derivee = (erreur - self.erreur_prec) / dt if dt > 0 else 0
        self.erreur_prec = erreur

        correction = self.Kp * erreur + self.Ki * self.integrale + self.Kd * derivee

        vit_mot_g = self.vitesse_base + correction - self.offset_mot
        vit_mot_d = self.vitesse_base - correction + self.offset_mot

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

pid = PIDCapController(Kp, Ki, Kd, vitesse_base, offset_mot)

# === Paramètres du mouvement circulaire ===
R1 = 230.0  # rayon (m)
v = 5.0      # vitesse (m/s)
T = 240.0    # durée totale (s)
lat_c, lon_c = 48.199706, -3.018784  # centre du cercle (°)

# --- Calcul de la vitesse angulaire correspondant à 1 m/s ---
# v = R * |omega|  =>  omega = -v / R
omega1 = -v / R1  # signe négatif = rotation horaire

# --- Échantillonnage temporel ---
t_vals = np.linspace(0, T, int(T) + 1)

# === Conversion locale -> géographique ===
def local_to_geo(x, y, lat_ref, lon_ref):
    """Convertit (x, y) en mètres -> (latitude, longitude) WGS84."""
    dlon = x / (111320 * np.cos(np.radians(lat_ref)))
    dlat = y / 110540.0
    return lat_ref + dlat, lon_ref + dlon

def R2(t,t0):
    return 50*np.exp(-(t-t0)/1000)+10






# ============================
# === BOUCLE PRINCIPALE GPS ===
# ============================

def mission(lat_target, lon_target, label="Bouée"):
    reference_x, reference_y = projDegree2Meter(lon_target, lat_target)
    distance = 999

    while distance > 4:
        gll_ok, gll_data = gps.read_gll_non_blocking()
        if not gll_ok:
            time.sleep(0.05)
            continue

        lat, lon = cvt_gll_ddmm_2_dd(gll_data)
        x, y = projDegree2Meter(lon, lat)
        dx = x - reference_x
        dy = y - reference_y
        distance = np.sqrt(dx * dx + dy * dy)

        heading_trigo = np.degrees(np.arctan2(dy, dx))
        heading_geo = (270 - heading_trigo) % 360

        print("[{}] lat={:.5f} lon={:.5f} dist={:.2f} heading_geo={:.1f}".format(label, lat, lon, distance, heading_geo))

        xmag, ymag, zmag = imu.read_mag_raw()
        Y = np.array([xmag, ymag, zmag])
        Y_corr = racineQ @ (Y - b)
        xmag_corr, ymag_corr, zmag_corr = Y_corr
        cap_act = arctan2(ymag_corr, xmag_corr) * 180 / np.pi

        cap_nav = convert_cap_sensor_to_nav(cap_act)
        print("cap_act = {}  cap_obj = {}".format(cap_nav, heading_geo))

        vg, vd = pid.correction_cap(heading_geo, cap_act)
        print("Moteurs: G={:.1f} D={:.1f}\n".format(vg, vd))

        ard.send_arduino_cmd_motor(vg, vd)
        enregistrer_valeurs(output_file, lat, lon, distance, cap_nav, heading_geo)
        kml.newpoint(name=label, coords=[(lon, lat)])

        time.sleep(0.1)

    print("Arrivé au point {} !".format(label))


# =======================
# === LANCEMENT MISSION ===
# =======================
try:
    for t in t_vals:
        x = R1 * np.cos(omega1 * t)
        y = R1 * np.sin(omega1 * t)
        lat, lon = local_to_geo(x, y, lat_c, lon_c)

        mission(lat, lon, label="Point t={:.1f}s".format(t))
finally:
    ard.send_arduino_cmd_motor(0, 0)
    kml.save("gps_data.kml")
    print("Données GPS enregistrées dans gps_data_foloow.kml")
    print("Mission terminée.\n")
