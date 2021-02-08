# from matplotlib import pyplot as mp
# import numpy as np

# def gaussian(x, mu, sig):
#     return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

# x_values = np.linspace(75, 200,120)
# for mu, sig in [(124, 11.97)]:
#     mp.plot(x_values, gaussian(x_values, mu, sig))

# mp.show()

from globalVariables import GlobalVariables
from kinematicRobo import kinematicRobo
from jacobianoPes import jacobianoPes
import numpy as np

glob = GlobalVariables()

theta = glob.getTheta()
hOrg = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1))
hP = hOrg
xe2 = kinematicRobo(theta,hOrg,hP,1,0)
Ja2 = jacobianoPes(theta,ha,xe2,0)