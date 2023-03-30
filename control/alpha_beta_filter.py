import random
import matplotlib.pyplot as plt
import numpy as np

import pandas as pd

pd.set_option('display.max_columns', None)
df = pd.read_csv('_slash_br5_slash_mavros_slash_imu_slash_water_pressure.csv')

def my_operation(x):
    return (x - 101300)/(1000*9.80665)

df['fluid_pressure'] = df['fluid_pressure'].apply(my_operation)
z = df['fluid_pressure'].to_list()


dt = 0.001

xk_0=z[0]
vk_0 = 0
a = 0.45
b = 0.1
# t = np.linspace(0,10,10000)
xk_1 = []
vk_1 = []
xm_0 = z

for xm in xm_0:
    # xm = random.random()*100
    # xm_0.append(xm)
    xk = xk_0 + vk_0*dt
    vk = vk_0

    rk = xm - xk
    xk +=a*rk
    vk += (b*rk)/dt

    xk_0 = xk
    vk_0 = vk 

    xk_1.append(xk_0)
    vk_1.append(vk_0)

plt.plot( xk_1, label='xk')
plt.plot( vk_1, label ='vk')
plt.plot( xm_0,label='xm_0')

plt.axhline(0, color='red')

plt.axvline(0, color='red')

plt.legend()

plt.show()