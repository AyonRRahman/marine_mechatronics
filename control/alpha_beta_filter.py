import random
import matplotlib.pyplot as plt
import numpy as np
from numpy.core.function_base import linspace
import pandas as pd

pd.set_option('display.max_columns', None)
df = pd.read_csv('_slash_br5_slash_mavros_slash_imu_slash_water_pressure.csv')
print(df.head())
def my_operation(x):
    return (x - 101300)/(1000*9.80665)

df['fluid_pressure'] = df['fluid_pressure'].apply(my_operation)
z = df['fluid_pressure'].to_list()

import pandas as pd
import numpy as np
from transforms3d.euler import quat2euler

# create a sample DataFrame with quaternion values

# create a function to convert quaternions to Euler angles
# def quaternion_to_euler(row):
#     q = np.array([row['x'], row['y'], row['z'], row['w']])
#     return pd.Series(quat2euler(q))

# df = pd.read_csv('_slash_br5_slash_mavros_slash_imu_slash_data.csv')
# # apply the function to each row of the DataFrame and store the results in a new column
# df[['roll', 'pitch', 'yaw']] = df.apply(quaternion_to_euler, axis=1)



# z = df['roll'].to_list()[3700:]

dt = 1/58

xk_0=z[0]
vk_0 = 0
a = 0.2
b = 0.01
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

plt.plot( z, label='yaw')
plt.plot( np.array(vk_1), label ='yaw rate predicted by the filter')
plt.grid()
# plt.plot(df['z.1'].to_list()[3700:],label = 'actual vel')
# plt.plot( xm_0,label='xm_0')

# plt.axhline(0, color='red')

# plt.axvline(0, color='red')
plt.yticks(np.core.linspace(0, 1, 9))
plt.legend(loc =0)

plt.show()