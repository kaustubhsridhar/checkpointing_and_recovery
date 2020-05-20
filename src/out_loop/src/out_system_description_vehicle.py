import numpy as np

# system description for diff drive mobile robot
class system_description():	
	def __init__(self):
		self.u0=np.array([[0], [0]]); self.x0=np.array([[0],[2],[0],[0]]); self.y0=self.x0; # initial values
		self.devID = np.array([[1],[1],[1],[1]]) # all sensors are not attacked at start
		self.u_type = "Pure Pursuit with Planner" # options are "Pure Pursuit" or "Pure Pursuit with Planner"
		self.Q = np.diag(np.array([0.1**2, 0.1**2, 0.01**2, 0.01**2])) # process cov matrix
		self.R = np.diag(np.array([0.1**2, 0.1**2, 0.01**2, 0.01**2])) # meas. cov matrix
		self.estimation_method="Kalman" # Kalman or none (for direct sensor measurments)
		self.T = 12 # run loop for 12 seconds
		self.CKPT_INT = 1 # create checkpoints every one second
		self.error_analysis = 0; # 1 if you want error analysis plot
		self.plot_curves = 1; # 1 if you want any plot

	def func_f(self,x,u,dt): # x_{k+1} = f_discrete(x_k,u_k) and x=[x,y,theta, v, delta], u = [a, phi]
		L = 0.45

		xp_dot = x[3,0]*np.cos(x[2,0])
		yp_dot = x[3,0]*np.sin(x[2,0])
		theta_dot = x[3,0]/L *np.tan(u[1,0])
		v_dot = u[0,0] 	

		xdot = np.array([[xp_dot], [yp_dot], [theta_dot], [v_dot]])
		x = x + xdot*dt

		return x

	def transform(self,u, u_prev, x, dt): # transform [a, phi] control to [wR, wL] control
		wR = (u[1,0] - u_prev[1,0])/ dt
		c_a = 1.633; c_m = 0.2; c_h = 4
		wL = (u[0,0] + c_a*x[3,0]) / (c_a*c_m) + c_h	
		return wR*1000, wL*1000

	def func_A(self,x,u):		# A = del f(x,u)/ del x where f is the continous dynamics
		L = 0.5
		return np.array([ [0,0,-x[3,0]*np.sin(x[2,0]),np.cos(x[2,0])], [0,0,x[3,0]*np.cos(x[2,0]),np.sin(x[2,0])], [0,0,0,np.tan(u[1,0])/L], [0,0,0,0] ])

		#return np.array([ [0,0,-x[3,0]*np.sin(x[2,0]+x[4,0]),np.cos(x[2,0]+x[4,0]),-x[3,0]*np.sin(x[2,0]+x[4,0])], [0,0,x[3,0]*np.cos(x[2,0]+x[4,0]),np.sin(x[2,0]+x[4,0]),x[3,0]*np.cos(x[2,0]+x[4,0])], [0,0,0,np.tan(x[4,0])/L,x[3,0]/(L*(np.cos(x[4,0]))**2)], [0,0,0,0,0], [0,0,0,0,0] ])

	def func_g(self,x,u=0):		# y = g(x,u)
		return x

	def func_C(self,x):		# C = del g/ del x
		return np.eye(len(x)) 

