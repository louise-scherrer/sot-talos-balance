import matplotlib.pyplot as plt
import numpy as np

from sys import argv
folder = argv[1]
if folder[-1]!='/':
    folder += '/'

comEst = np.loadtxt(folder+"dg_cdc_estimator-c.dat")
comRef = np.loadtxt(folder+"dg_comAdmCtrl-comRef.dat")
comDes = np.loadtxt(folder+"dg_dummy_wp-comDes.dat")
has_dyn= True
try:
    comDyn = np.loadtxt(folder+"dg_robot_dynamic-com.dat")
except:
    print("No dyn")
    has_dyn= False

zmpRef = np.loadtxt(folder+"dg_dcmCtrl-zmpRef.dat")
zmpEst = np.loadtxt(folder+"dg_zmpEst-zmp.dat")

#plt.ion()

plt.figure()
plt.suptitle('CoM')

plt.subplot(1,3,1)
plt.plot(comDes[:,0],comDes[:,1])
plt.plot(comRef[:,0],comRef[:,1])
plt.plot(comEst[:,0],comEst[:,1])
if has_dyn:
    plt.plot(comDyn[:,0],comDyn[:,1])
leg = plt.legend(['des','ref','est','dyn'])
leg.draggable()
plt.title('x')

plt.subplot(1,3,2)
plt.plot(comDes[:,0],comDes[:,2])
plt.plot(comRef[:,0],comRef[:,2])
plt.plot(comEst[:,0],comEst[:,2])
if has_dyn:
    plt.plot(comDyn[:,0],comDyn[:,2])
leg = plt.legend(['des','ref','est','dyn'])
leg.draggable()
plt.title('y')

plt.subplot(1,3,3)
plt.plot(comDes[:,0],comDes[:,3])
plt.plot(comRef[:,0],comRef[:,3])
plt.plot(comEst[:,0],comEst[:,3])
if has_dyn:
    plt.plot(comDyn[:,0],comDyn[:,3])
leg = plt.legend(['des','ref','est','dyn'])
leg.draggable()
plt.title('z')

plt.figure()
plt.suptitle('ZMP')

plt.subplot(1,2,1)
plt.plot(zmpRef[:,0],zmpRef[:,1])
plt.plot(zmpEst[:,0],zmpEst[:,1])
leg = plt.legend(['ref','est'])
leg.draggable()
plt.title('x')

plt.subplot(1,2,2)
plt.plot(zmpRef[:,0],zmpRef[:,2])
plt.plot(zmpEst[:,0],zmpEst[:,2])
leg = plt.legend(['ref','est'])
leg.draggable()
plt.title('y')

plt.show()
