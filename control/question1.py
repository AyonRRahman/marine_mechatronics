import numpy as np
import matplotlib.pyplot as plt

a = np.linspace(1,100,10000).reshape(1,10000)
b = np.linspace(100,200,10000).reshape(1,10000)

# b.shape
c = np.concatenate((a,b),axis=1)
plt.plot(c.T)


plt.show()
