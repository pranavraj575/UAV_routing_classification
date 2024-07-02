# plots info gain wrt time spent

import numpy as np
import os
from matplotlib import pyplot as plt

tau=.5
max_rng=15
num=100

def Pi(d):
    return 1-.5*np.exp(-np.sqrt(d/tau))
def Ii(d):
    return Pi(d)*np.log(Pi(d))+(1-Pi(d))*np.log(1-Pi(d))+np.log(2)

grid=np.array(list(range(num)))/num*max_rng
plt.plot(grid,[Ii(d) for d in grid])

plot_folder=os.path.join("output","plots")
if not os.path.exists(plot_folder):
    os.makedirs(plot_folder)
plt.savefig(os.path.join(plot_folder, 'info_gain.png'))
plt.show()
