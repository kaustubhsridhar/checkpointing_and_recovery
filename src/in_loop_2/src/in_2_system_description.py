import numpy as np
from scipy import signal

# system description for dc motor
class system_description():	
	def __init__(self):
		R = 1; L = 0.5; K = 0.01; J = 0.01;
		self.A = np.array([[-R/L, -K/L],[K/J, -K/J]]); self.B = np.array([[1/L],[0]]); self.C = np.array([[0,1]]); self.D = np.array([[0]]) # A, B, C, D matrices of LTI system
		self.x0=np.array([[0],[0]]); self.u0=np.array([[0.001]]); self.y0=np.array([[0]]); # initial values
		self.devID = np.array([[1],[1]]) # all 2 sensors are not attacked at start
		self.u_type = "PID" # PID DC motor control
		self.Q = (200**2) * np.eye(len(self.x0)) # process cov matrix
		self.R = (200**2) * np.eye(len(self.y0)) # measurement cov matrix
		self.estimation_method="Kalman" # Kalman or none (for direct sensor measurments)
		self.T = 12 # run loop for 12 seconds
		self.CKPT_INT = 1 # create checkpoints every one second
		self.error_analysis = 1; # 1 if you want error analysis plot

	def func_f(self,x,u,dt):	# x_{k+1} = f(x_k,u_k) and x=[x,y,theta]
		Mats = signal.cont2discrete((self.A,self.B,self.C,self.D), dt)
		A_d = Mats[0]
		B_d = Mats[1]
		return np.matmul(A_d, x) + np.matmul(B_d, u)

	def func_A(self,x,u):		# A = del f(x,u)/ del x
		return self.A

	def func_g(self,x,u=0):		# y = g(x,u)
		return np.matmul(self.C, x)

	def func_C(self,x):		# C = del g/ del x
		return self.C

