import casadi as ca
import numpy as np
import math
import control as ct

def mincopter_dynamics(mass=1,
                       g=9.81,
                       ix=0.008,
                       iy=0.015,
                       iz=0.017,
                       l=0.2,
                       dt=0.01):
    '''Returns the full dynamics (as a casadi function) as well as the discretised andlinearised quadrotor
    dynamics equations, based on a set of input configuration arguments'''
    
    ## State and Input
    x = ca.SX.sym("x", 12)
    u = ca.SX.sym("u", 4)
    
    x_x, x_y, x_z, x_phi, x_theta, x_psi, x_dx, x_dy, x_dz, x_dphi, x_dtheta, x_dpsi = ca.vertsplit(x, 1)
    u_F, u_Tx, u_Ty, u_Tz = ca.vertsplit(u, 1)

    ## Dynamics Equations
    dx_x = x_dx
    dx_y = x_dy
    dx_z = x_dz
    dx_phi = x_dphi
    dx_theta = x_dtheta
    dx_psi = x_dpsi
    dx_dx = -(u_F/mass) * (ca.cos(x_phi)*ca.sin(x_theta)*ca.cos(x_psi) + ca.sin(x_phi)*ca.sin(x_psi))
    dx_dy = -(u_F/mass) * (ca.cos(x_phi)*ca.sin(x_theta)*ca.sin(x_psi) - ca.sin(x_phi)*ca.cos(x_psi))
    dx_dz = -(u_F/mass) * (ca.cos(x_phi) * ca.cos(x_theta)) + g
    dx_dphi = 1/ix * (u_Tx + x_dtheta * x_dpsi*(iy - iz))
    dx_dtheta = 1/iy * (u_Ty + x_dpsi*x_dphi*(iz - ix))
    dx_dpsi = 1/iz * (u_Tz + x_dphi*x_dtheta*(ix - iy))
    
    x_dot = ca.vertcat(dx_x, dx_y, dx_z, dx_phi, dx_theta, dx_psi, dx_dx, dx_dy, dx_dz, dx_dphi, dx_dtheta, dx_dpsi)
    f = x_dot

    dynamics = ca.Function("quadrotor_dyn", [x, u], [x + dt * x_dot])
    
    ## NOTE Optional Testing
    '''
    ## Apply 25N of force for 10 seconds
    out = [0, 0, -20, 0.2, 0.2, 0, 0, 0, 0, 0, 0, 0]
    u_in = [25, 0, 0, 0]
    for i in range(0,1000):
        out = dynamics(out, u_in)
    print(f"State after 10s: {out}")
    
    assert(out[0]<0) ## Negative x-coordinate with +ve pitch
    assert(out[1]>0) ## Positive y-coordinate with +ve roll
    assert(out[2]<0) ## Negative z-coordinate with positive net Force in NED frame
    '''
    
    ## Linearization
    x_operating = np.zeros((12, 1))

    ## NOTE We are linearising here about the hovering point (at which our input force, F, is equal to m*g)
    u_operating = np.array([mass*g, 0, 0, 0]).reshape((-1, 1))  # hovering (mg 0 0 0)
    
    A = ca.Function("A", [x, u], [ca.jacobian(f, x)])(x_operating, u_operating)
    B = ca.Function("B", [x, u], [ca.jacobian(f, u)])(x_operating, u_operating)
    C = np.eye(12)
    D = np.zeros((12, 4))
    
    # Create numerical matrices and cast into numpy array
    A = np.array(ca.DM(A))
    B = np.array(ca.DM(B))
    
    lin_ss_cont = ct.ss(A,B,C,D)
    lin_ss_disc = ct.c2d(lin_ss_cont,dt,method='zoh')

    return dynamics, lin_ss_disc

