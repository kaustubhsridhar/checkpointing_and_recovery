import numpy as np

# system description for diff drive mobile robot
class system_description():	
	def __init__(self):
		self.u0=np.array([[0], [0]]); self.x0=np.array([[0],[0],[0]]); self.y0=self.x0; # initial values
		self.devID = np.array([[1],[1],[1]]) # all 3 sensors are not attacked at start
		self.u_type = "linear" # linear dynamic inversion control
		self.Q = (0.1**2) * np.eye(len(self.x0)) # process cov matrix
		self.R = (0.1**2) * np.eye(len(self.y0)) # measurement cov matrix
		self.estimation_method="Kalman" # Kalman or none (for direct sensor measurments)
		self.T = 12 # run loop for 12 seconds
		self.CKPT_INT = 1 # create checkpoints every one second

	def func_f(self,x,u,dt):	# x_{k+1} = f(x_k,u_k) and x=[x,y,theta]
		xdot = np.array([[u[0,0]*np.cos(x[2,0])], [u[0,0]*np.sin(x[2,0])], [ u[1,0] ]])
		x = x + xdot*dt
		x[2,0] = np.arctan(np.sin(x[2,0]) / np.cos(x[2,0]))
		return x

	def transform(self,u):		# transform [v, w] control to [wR, wL] control
		R = 0.05; L = 0.5;
		v = u[0,0]; w = u[1,0]
		wR = (2*v + w*L) / (2*R)
		wL = (2*v - w*L) / (2*R)
		return wR, wL

	def func_A(self,x,u):		# A = del f(x,u)/ del x
		return np.array([ [0,0,-u[0,0]*np.sin(x[2,0])], [0,0,u[0,0]*np.cos(x[2,0])], [0,0,0] ])

	def func_g(self,x,u=0):		# y = g(x,u)
		return x

	def func_C(self,x):		# C = del g/ del x
		return np.eye(len(x)) 

