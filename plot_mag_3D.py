import numpy as np
import scipy as sp
import matplotlib.pyplot as plt



# --- 1. Lecture + conversion µT ---
data_raw = np.loadtxt("mag_raw_data.txt", skiprows=1)
data = data_raw / 68.42  # µT
n = data.shape[0]

# --- 2. Préparation des données ---
x, y, z = data[:,0], data[:,1], data[:,2]

# --- 3. Calcul du vecteur des paramètres ---
M = np.zeros((n,9))
for i in range(n):
    xi, yi, zi = x[i], y[i], z[i]
    M[i,:] = [xi**2, yi**2, zi**2, xi*yi, xi*zi, yi*zi, xi, yi, zi]

# Résolution par les moindres carrés
MtM = M.T.dot(M)
MtM_inv = np.linalg.inv(MtM)
MtY = M.T.dot(np.ones(n))
p = MtM_inv.dot(MtY)

# --- 4. Extraction des coefficients ---
Q = np.array([[p[0], p[3]/2, p[4]/2],
              [p[3]/2, p[1], p[5]/2],
              [p[4]/2, p[5]/2, p[2]]])

b = -0.5 * Q @ np.array([[p[6]],
                     [p[7]],
                     [p[8]]])
b = np.array([[b[0], b[1], b[2]]])
data_corr = np.zeros((3,n))

for i in range(n):
    xi, yi, zi = x[i], y[i], z[i]
    vec = np.array([[xi], [yi], [zi]])
    data_corr[:, i] = (sp.linalg.sqrtm(Q) @ (vec - b)).ravel()

x_corr,y_corr,z_corr = data_corr[0,:], data_corr[1,:], data_corr[2,:]

# --- 7. Visualisation ---
fig = plt.figure(figsize=(12,6))

ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(x, y, z, c='r', s=8)
ax1.set_title("Nuage brut (µT)")
ax1.set_xlabel("Mag X [µT]")
ax1.set_ylabel("Mag Y [µT]")
ax1.set_zlabel("Mag Z [µT]")
ax1.grid(True)

ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(x_corr, y_corr, z_corr, c='r', s=8)
ax2.set_title("Nuage corrigé (µT)")
ax2.set_xlabel("Mag X [µT]")
ax2.set_ylabel("Mag Y [µT]")
ax2.set_zlabel("Mag Z [µT]")
ax2.grid(True)
plt.show()


"""
ax2.scatter(x, y, z, c='r', s=8)

# Création surface ellipsoïde
u = np.linspace(0, 2*np.pi, 40)
v = np.linspace(0, np.pi, 20)
xe = radii[0]*np.outer(np.cos(u), np.sin(v))
ye = radii[1]*np.outer(np.sin(u), np.sin(v))
ze = radii[2]*np.outer(np.ones_like(u), np.cos(v))
E = np.stack((xe, ye, ze), axis=-1).dot(eigvecs.T) + center

ax2.plot_surface(E[:,:,0], E[:,:,1], E[:,:,2], color='cyan', alpha=0.5, linewidth=0)
ax2.set_title("Ellipsoïde ajustée (fit analytique stable)")
ax2.set_xlabel("Mag X [µT]")
ax2.set_ylabel("Mag Y [µT]")
ax2.set_zlabel("Mag Z [µT]")
ax2.grid(True)

plt.tight_layout()"""


