import numpy as np
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
plt.savefig("plots//info_gain.png")
plt.show()
