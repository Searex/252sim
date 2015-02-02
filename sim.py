# Chris Lorenzen - University of California, Davis
import numpy as np
#import matplotlib.pyplot as plt

# Declare Constants
a_t = 1 # semi-major axis [m]
mu = 1 # gravitational parameter [m^3/s^2]
n = np.sqrt(mu/a_t**3)

# Simulation Parameters
state_initial = [-10,1,0,10,0,0] # x [m] , y [m], z [m], x_dot [m/s], y_dot [m/s], z_dot [m/s]
state_target = [0,1,0,0,0,0] # x [m] , y [m], z [m], x_dot [m/s], y_dot [m/s], z_dot [m/s]
time_final = 1
time_step = 10**-1
time_initial = 0
i_max=(time_final-time_initial)/time_step+1

# Simulation
t = time_initial
state=state_initial
i=0
states = np.zeros((i_max,6))
while t <= time_final:
	sin = np.sin(n*t)
	cos = np.cos(n*t)
	A11 = 4-3*cos
	A14 = 1/n*sin
	A15 = 2/n*(1-cos)
	A21 = 6*(sin-n*t)
	A24 = -2/n*(1-cos)
	A25 = 1/n*(4*sin-3*n*t)
	A33 = cos
	A36 = 1/n*sin
	A41 = 3*n*sin
	A44 = cos
	A45 = 2*sin
	A51 = -6*n*(1-cos)
	A54 = -2*sin
	A55 = 4*cos-3
	A63 = -n*sin
	A66 = cos
	A = [[A11,0,0,A14,A15,0],[A21,1,0,A24,A25,0],[0,0,A33,0,0,A36],[A41,0,0,A44,A45,0],[A51,0,0,A54,A55,0],[0,0,A63,0,0,A66]]
	state_new = np.dot(A,state)
	states[i] = state
	state = state_new
	t = t+time_step
	i = i+1
	print t

print states