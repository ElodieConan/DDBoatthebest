import sys
import os
import time
import numpy as np
from numpy.ma.core import arctan2

#access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv
offset_mot = 0
k=0.5  #gain de correction
ard= arddrv.ArduinoIO() #create an ARduino object
imu = imudrv.Imu9IO()


def convert_cap_sensor_to_nav(cap_sensor):
    """
    Convertit le cap du capteur (N=±180°, S=0°, E=90°, O=-90°)
    vers le système de navigation (N=0°/360°, E=90°, S=180°, O=270°).
    """
    cap_nav = (cap_sensor + 180) % 360
    return cap_nav


def correction_cap(vitesse_base, cap_obj, cap_act_sensor, offset_mot=0):
    """
    Calcule les vitesses moteur gauche/droite pour corriger le cap.
    Les angles sont dans le système NAV :
        - N = 0° / 360°
        - E = 90°
        - S = 180°
        - O = 270°

    Entrées :
        vitesse_base : vitesse moyenne des moteurs (0 à 255)
        cap_obj : cap cible en degrés NAV
        cap_act_sensor : cap mesuré (format capteur)
        offset_mot : correction mécanique entre moteurs

    Sorties :
        vit_mot_g, vit_mot_d (bornées entre -255 et 255)
    """

    # --- Conversion du cap capteur en cap navigation ---
    cap_act = convert_cap_sensor_to_nav(cap_act_sensor)

    # --- Calcul erreur de cap dans [-180, 180] ---
    erreur = cap_obj - cap_act
    if erreur > 180:
        erreur -= 360
    elif erreur < -180:
        erreur += 360

    # --- Normalisation dans [-1, 1] ---
    erreur_norm = erreur / 180.0

    # --- Commande proportionnelle ---
    correction = 120 * erreur_norm

    # --- Vitesses moteur gauche / droite ---
    vit_mot_g = vitesse_base - correction - offset_mot
    vit_mot_d = vitesse_base + correction + offset_mot

    # --- Saturation dans [-255, 255] ---
    vit_mot_g = max(min(vit_mot_g, 255), 0)
    vit_mot_d = max(min(vit_mot_d, 255), 0)

    return vit_mot_g, vit_mot_d



def angles_Euler(a1,y1):
    r1=y1-((y1.T@a1)*a1)
    normr1=np.linalg.norm(r1)
    v = (y1 - (y1.T @ a1) * a1).flatten()
    r2 = np.cross(a1.flatten(), v)
    normr2=np.linalg.norm(r2)
    R = np.column_stack((r1 / normr1, r2 / normr2, a1))
    phi=np.arctan2(R[2,1],R[2,2])
    theta=-np.arcsin(R[2,0])
    psi=np.arctan2(R[1,0],R[0,0])
    return phi,theta,psi


# transformation des données mag en cap
#Données de correction :

p= np.array([ 9.19344322e-08,  8.86431491e-08,  1.40532829e-08 , 2.99160959e-08,
 -2.11090640e-08,  1.65789833e-08 , 3.76555627e-05, -7.74120350e-05,
 -6.36399910e-05])
racineQ= np.array([[ 3.00888414e-04,  2.60587871e-05, -2.68613861e-05],
 [ 2.60587871e-05,  2.95772384e-04,  2.19723862e-05],
 [-2.68613861e-05,  2.19723862e-05,  1.13353267e-04]])

Q = np.array([[ 9.19344322e-08,  1.49580480e-08, -1.05545320e-08],
 [ 1.49580480e-08 , 8.86431491e-08 , 8.28949163e-09],
 [-1.05545320e-08,  8.28949163e-09 , 1.40532829e-08]])

# --- Calcul du vecteur b ---
b = -0.5 * np.array([
    (1/Q[0,0]) * p[6],
    (1/Q[1,1]) * p[7],
    (1/Q[2,2]) * p[8]
])


a0 = np.array([[0], [0], [1]])

a1 = a0  # hypothèse si il n'y a pas de vagues, à améliorer pour après

while True:
    # --- Lecture du magnétomètre ---
    xmag, ymag, zmag = imu.read_mag_raw()

    # --- Mise sous forme de vecteur colonne ---
    Y = np.array([xmag, ymag, zmag])
    Y_corr = racineQ @ (Y - b)

    # --- Résultats corrigés ---
    xmag_corr, ymag_corr, zmag_corr = Y_corr
    Y1 = Y_corr.reshape(3,1)  # même format que ton ancienne version
    #acceleration
    R_inu = np.array([[-0.5544297, 0.12721476, 0.00438503],
                      [0.07167656, 0.59082185, -0.10687039],
                      [0.45592243, 0.32278327, -0.53408991]])

    #Y1 = R_inu @ Y1



    #cap_act=angles_Euler(a1,Y1)[2]*180/np.pi
    cap_act=arctan2(ymag_corr, xmag_corr)*180/np.pi
    print("cap after rot:", convert_cap_sensor_to_nav(cap_act))
    left_speed, right_speed = correction_cap(110, 200, cap_act)
    print(left_speed, right_speed)
    ard.send_arduino_cmd_motor(left_speed, right_speed) #in place turn
