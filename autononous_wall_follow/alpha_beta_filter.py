import random
import matplotlib.pyplot as plt
import numpy as np
from numpy.core.function_base import linspace
import pandas as pd

pd.set_option('display.max_columns', None)
df = pd.read_csv('_slash_br5_slash_mavros_slash_imu_slash_water_pressure.csv')

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
class AlphaBetaFilter:
    def __init__(self,initial_depth,a=0.1,b=0.005,dt=1/58):
        self.a = a
        self.b = b
        self.dt = dt
        self.vk=0
        self.xk=0
        self.xk_0=initial_depth
        self.vk_0=0

    def filter_step(self, xm):
        self.xk = self.xk_0 + self.vk_0*self.dt
        self.vk = self.vk_0

        self.rk = xm - self.xk
        self.xk +=self.a*self.rk
        self.vk += (self.b*self.rk)/self.dt

        self.xk_0 = self.xk
        self.vk_0 = self.vk 
        
        return self.vk_0

alphabeta = AlphaBetaFilter(initial_depth=z[0])


vks=[]
for depth in z[1:]:
    vks.append(alphabeta.filter_step(depth))


plt.plot( z, label='depth')
plt.plot( np.array(vks), label ='depth rate predicted by the filter')
plt.grid()
# plt.plot(df['z.1'].to_list()[3700:],label = 'actual vel')
# plt.plot( xm_0,label='xm_0')

# plt.axhline(0, color='red')

# plt.axvline(0, color='red')
plt.yticks(np.core.linspace(0, 1, 9))
plt.legend(loc =0)

plt.show()