import numpy as np
import scipy as sp
import matplotlib.pyplot as plt



# --- 1. Lecture + conversion µT ---
data_raw = np.loadtxt("mag_raw_data.txt", skiprows=1)


def ell_in2_sph(data):
    n = data.shape[0]
    # --- 2. Préparation des données ---
    x, y, z = data[:,0], data[:,1], data[:,2]

    # --- 3. Calcul du vecteur des paramètres ---
    M = np.zeros((n,9))
    for i in range(n):
        xi, yi, zi = x[i], y[i], z[i]
        M[i,:] = [xi**2, yi**2, zi**2, xi*yi, xi*zi, yi*zi, xi, yi, zi]

    # Résolution par les moindres carrés
    p = np.linalg.inv(M.T @ M) @ M.T @ np.ones(n)
    res = np.ones(n)- M.dot(p)
    print(res)

    print("Vecteur des paramètres p :\n", p)
    # --- 4. Extraction des coefficients ---
    Q = np.array([[p[0], p[3]/2, p[4]/2],
                  [p[3]/2, p[1], p[5]/2],
                  [p[4]/2, p[5]/2, p[2]]])


    print("Matrice Q :\n", Q)

    b = -0.5 * np.linalg.inv(Q) @ np.array([[p[6]],
                         [p[7]],
                         [p[8]]])

    data_corr = np.zeros((3,n))

    for i in range(n):
        xi, yi, zi = x[i], y[i], z[i]
        vec = np.array([[xi], [yi], [zi]])
        data_corr[:, i]= (sp.linalg.sqrtm(Q) @ (vec - b)).ravel()


    return data_corr[0,:], data_corr[1,:], data_corr[2,:]

# --- 7. Visualisation ---
# Nombre de vecteurs
N = 1000

# Tirage de points aléatoires selon une distribution normale
vecs = np.random.randn(N, 3)   # chaque ligne = [x, y, z]

# Normalisation (chaque vecteur a une norme = 1)
norms = np.linalg.norm(vecs, axis=1, keepdims=True)
vecs_normalized = vecs / norms


fig = plt.figure(figsize=(12,6))

ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(data_raw[:,0], data_raw[:,1], data_raw[:,2], c='r', s=8)
ax1.set_title("Nuage brut (µT)")
ax1.set_xlabel("Mag X [µT]")
ax1.set_ylabel("Mag Y [µT]")
ax1.set_zlabel("Mag Z [µT]")
ax1.grid(True)

x_corr, y_corr, z_corr = ell_in2_sph(data_raw)
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(x_corr, y_corr, z_corr, c='b', s=8)
ax2.set_title("Nuage corrigé (µT)")
ax2.set_xlabel("Mag X [µT]")
ax2.set_ylabel("Mag Y [µT]")
ax2.set_zlabel("Mag Z [µT]")
ax2.grid(True)

# --------------------------------------------------------------------
# === AJOUT : Fit ellipsoïdal sur les nuages brut et corrigé ===
# --------------------------------------------------------------------
def fit_ellipsoid(data):
    x, y, z = data[:,0], data[:,1], data[:,2]
    n = data.shape[0]
    M = np.column_stack([x**2, y**2, z**2, x*y, x*z, y*z, x, y, z])
    p, *_ = np.linalg.lstsq(M, np.ones(n), rcond=None)

    Q = np.array([
        [p[0], p[3]/2, p[4]/2],
        [p[3]/2, p[1], p[5]/2],
        [p[4]/2, p[5]/2, p[2]]
    ])
    Q = 0.5 * (Q + Q.T)

    b = (-0.5 * np.linalg.inv(Q) @ np.array([p[6], p[7], p[8]])).ravel()

    val = b @ Q @ b - 1
    Qn = Q / val
    eigvals, eigvecs = np.linalg.eigh(Qn)
    radii = 1.0 / np.sqrt(np.abs(eigvals))
    sph_ratio = np.min(radii) / np.max(radii)
    return b, radii, eigvecs, sph_ratio

def plot_ellipsoid(ax, center, radii, eigvecs, color='cyan', alpha=0.3):
    u = np.linspace(0, 2*np.pi, 40)
    v = np.linspace(0, np.pi, 20)
    xe = radii[0]*np.outer(np.cos(u), np.sin(v))
    ye = radii[1]*np.outer(np.sin(u), np.sin(v))
    ze = radii[2]*np.outer(np.ones_like(u), np.cos(v))
    E = np.stack((xe, ye, ze), axis=-1).dot(eigvecs.T) + center
    ax.plot_surface(E[:,:,0], E[:,:,1], E[:,:,2], color=color, alpha=alpha, linewidth=0)

# --- Données brutes ---
b_raw, radii_raw, eigvecs_raw, sph_raw = fit_ellipsoid(data_raw)
print("\n=== Fit ellipsoïdal sur les données BRUTES ===")
print("Centre :", np.round(b_raw, 3))
print("Rayons :", np.round(radii_raw, 3))
print(f"Facteur de sphéricité : {sph_raw:.4f}")

plot_ellipsoid(ax1, b_raw, radii_raw, eigvecs_raw, color='orange', alpha=0.4)

# --- Données corrigées ---
data_corr_plot = np.vstack((x_corr, y_corr, z_corr)).T
b_corr, radii_corr, eigvecs_corr, sph_corr = fit_ellipsoid(data_corr_plot)
print("\n=== Fit ellipsoïdal sur les données CORRIGÉES ===")
print("Centre :", np.round(b_corr, 3))
print("Rayons :", np.round(radii_corr, 3))
print(f"Facteur de sphéricité : {sph_corr:.4f}")

plot_ellipsoid(ax2, b_corr, radii_corr, eigvecs_corr, color='lime', alpha=0.4)


plt.show()

