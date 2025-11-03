import sys
import os
import time

# --- Accès aux drivers du DDBoat ---
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv

# --- Création de l'objet IMU ---
imu = imudrv.Imu9IO()

# --- Fichier de sortie ---
output_file = "mag_raw_data.txt"

# --- Nombre de mesures à acquérir ---
N = 1000

print("Début de l'acquisition magnétomètre...")
print("Fichier :", output_file)
print("Bouge lentement le DDBoat dans toutes les directions.")

with open(output_file, "w") as f:
    f.write("xmag ymag zmag\n")  # en-tête
    for i in range(N):
        xmag, ymag, zmag = imu.read_mag_raw()
        line = "{} {} {}\n".format(xmag, ymag, zmag)
        f.write(line)
        if i % 50 == 0:
            print("  {}/{} mesures enregistrées...".format(i, N))
        time.sleep(0.01)

print("✅ Acquisition terminée. Données sauvegardées dans", output_file)
