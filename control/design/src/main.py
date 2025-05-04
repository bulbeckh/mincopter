
import control as ct
from control.matlab import c2d

from matplotlib import pyplot as plt

import numpy as np

## state space model

g = 9.8
m=1
l=0.2
I_x=0.11
I_y=0.11
I_z=0.04

linearized_A = np.array(
    [
#    x  y  z  p  t  p xd yd zd pd td pd
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], # d_x
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], # d_y
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0], # d_z
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], # d_phi
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], # d_theta
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # d_psi
    ## 2nd block - dd_x, dd_y, dd_z
    [0, 0, 0, 0, g, 0, 0, 0, 0, 0, 0, 0], # dd_x
    [0, 0, 0, g, 0, 0, 0, 0, 0, 0, 0, 0], # dd_y
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # dd_z
    ## 3rd block - dd_phi, dd_theta, dd_psi
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # dd_phi
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # dd_theta
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]] # dd_psi
)

linearized_B = np.array(
    [[0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    ## 2nd block
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [1/m, 0, 0, 0],
    ## 3rd block
    [0, l/I_x, 0, 0],
    [0, 0, l/I_y, 0],
    [0, 0, 0, l/I_z]]
)

C = np.identity(12)
D = np.zeros((12,4))

ss = ct.StateSpace(linearized_A, linearized_B, C, D)

disc = c2d(ss, 0.1)

print(disc)

## Response

resp = ct.initial_response(disc, X0=[0,0,15,0,0,0,0,0,0,0,0,0])

resp_t = resp.time
resp_s = resp.states

plt.plot(resp_t, resp_s[0], 'b',
         resp_t, resp_s[1], 'r',
         resp_t, resp_s[2], 'r',
         resp_t, resp_s[3], 'r',
         resp_t, resp_s[4], 'r',
         resp_t, resp_s[5], 'r'
         )

plt.show()


