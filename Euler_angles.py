from roblib import *

#Correction des données :

p=  [ 2.76555971e-07 , 2.79324789e-07 , 2.30959703e-07 , 4.30784536e-08,
 -3.36328677e-08,  2.78418943e-08,  2.87369877e-04, -3.43639329e-04,
  1.27332450e-03]

Q=np.array([[ 2.76555971e-07,  2.15392268e-08, -1.68164339e-08],
 [ 2.15392268e-08 , 2.79324789e-07 , 1.39209471e-08],
 [-1.68164339e-08 , 1.39209471e-08 , 2.30959703e-07]])

racineQ = np.array( [[ 5.25203370e-04 , 2.06816012e-05 ,-1.70194899e-05],
 [ 2.06816012e-05,  5.27917190e-04,  1.41598059e-05],
 [-1.70194899e-05,  1.41598059e-05 , 4.80072432e-04]])

b = -0.5 * np.linalg.inv(Q) @ np.array([[p[6]],
                                        [p[7]],
                                        [p[8]]])
b=b.flatten()

raw_data = np.loadtxt("mag_raw_data_static.txt", skiprows=1)

n = raw_data.shape[0]
data=np.zeros((n,3))
for i in range(n):
    data[i,:]=racineQ@(raw_data[i,:]-b)

#Fonctions pour obtenir les angles d'Euler :

#façon exo 8 inertiel

def bank(a1): #roulis
    return np.arcsin(a1[1,0])
def elevation(a1): #tanguage
    return np.arcsin(a1[0,0])
def heading(a1, y1) : #lacet
    Rh=rotuv(a1,a0)
    yh=Rh@y1
    return -np.arctan2(yh[1],yh[0])

#façon cours

def angles_Euler(a1,y1):
    R=np.array([(y1@y1.T)@a1])

x, y, z = data[:, 0], data[:, 1], data[:, 2]
y1_corr = np.column_stack((x, y, z))
a0 = array([[0], [0], [1]])

a1 = a0  # hypothèse si il n'y a pas de vagues, à améliorer pour après

n = data.shape[0]
x, y, z = data[:,0], data[:,1], data[:,2]
y1_corr=np.column_stack((x, y,z))
for i in range(n):
    y1_corr = np.array([x[i], y[i], z[i]])
    print("cap",heading(a1,y1_corr.T)*180/np.pi)