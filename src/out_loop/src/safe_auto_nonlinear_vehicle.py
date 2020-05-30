import numpy as np
from scipy.linalg import expm
from matplotlib import pyplot as plt
# ddrive class (here) inherits from base class
from safe_auto_nonlinear_base import safe_auto_nonlinear 

class safe_auto_nonlinear_ddrive(safe_auto_nonlinear):
    def __init__(self, func_f, func_A, func_g, func_C, x0, u0, y0, interval, T, EM, u_type, devID, Q, R, EA):
	super(safe_auto_nonlinear_ddrive, self).__init__(func_f, func_A, func_g, func_C, x0, u0, y0, interval, T, EM, u_type, devID, Q, R, EA) # brings everything over from parent class
	# add new variables for this child class below
	self.e = 0; self.eder = 0; self.esum = 0 
	self.time_to_CkPt_val = 0; self.NOW_old = 0
	self.waypt_list = []
        

    def anomaly_detect(self):
        # attack detection algo goes here
        if self.NOW>=self.s1 and self.NOW <self.e1+0.1:
            self.devID = np.array([[0],[0],[1],[1]])
        elif self.NOW>=self.s2 and self.NOW <self.e2+0.1:
            self.devID = np.array([[0],[0],[1],[1]])
        else:
            self.devID = np.array([[1],[1],[1],[1]])

        # check to see if any devices are BAD(0); if not, return NO(0) anomaly
        self.prev_check = self.check
        if np.any(self.devID==0)==0:
            self.check = 0
        else:
            self.check = 1
        self.change = self.check - self.prev_check
        # at loc of every attack start, increment count by 1
        if self.NOW>=self.s1:self.attk_ct = 1
        if self.NOW>=self.s2:self.attk_ct = 2
        
        return self.check


    def Find_Waypoint(self, x, y, L):
	if self.u_type == "Pure Pursuit":
		C = 2.1 # tune
		Ld = 1 # dummy value; calculated later
		xref = 2*np.sin(self.NOW/C); yref = 2*np.cos(self.NOW/C)

	elif self.u_type == "Pure Pursuit with Planner":
		# generating list of tuples of waypoints at start of run
		A = 4; C = 2.1 # tune
		waypt_list_created = 0
		if self.NOW <= self.dt and waypt_list_created == 0:
			waypt_list_created = 1
			for ct in range(1, int((self.T+2)/0.1)):
				self.waypt_list.append( (A*np.sin(ct*0.1/C), A*np.cos(ct*0.1/C)) )
			#print(waypt_list)

		# return waypoint at Look ahead distance or just beyond
		Ld = 0.94*A/2 # tune
		for tup in self.waypt_list[int(self.NOW/0.1):]:
			dist = ((tup[0] - x)**2 + (tup[1] - (y))**2)**(0.5)
		
			if dist>=Ld:
			
				xref = tup[0]; yref = tup[1]
				#print(tup[0], tup[1])
				break

	xref_dot = yref/C; yref_dot = -xref/C
	xref_ddot = yref_dot/C; yref_ddot = -xref_dot/C
	vref = (xref_dot**2 + yref_dot**2)**(0.5)
	vref_dot = 1/(xref_dot**2 + yref_dot**2)**(0.5) * (xref_dot*xref_ddot +yref_dot*yref_ddot)
	theta_ref = np.arctan2(yref_dot/vref, xref_dot/vref)

	return xref, yref, vref, vref_dot, theta_ref, Ld


    def StartControl(self): 
        
        # save value predicted after rollfwd
        self.predicted.append(self.xe[:,0]) #takes first column of np.array
        
        # take old values
        eold = self.e; 

        # Pure Pursuit (with and without planner)
        if "Pure Pursuit" in self.u_type: 
		L = 0.45
		Kp = 6.6/1; Ki = 3; Kd = 0.091;

		# current states
		x = self.xe[0,0]; y = self.xe[1,0]; theta = self.xe[2,0]; v = self.xe[3,0]
		
		# ref trajectory's waypoint, dervs. and Look-ahead distance
		xref, yref, vref, vref_dot, theta_ref, Ld = self.Find_Waypoint(x, y, L)
		
		# transform goal point to vehicle frame of reference
		xref_t = xref - x
		yref_t = yref - y
		xref_tf = np.cos(-(90-theta))*xref_t + np.sin(-(90-theta))*yref_t
		yref_tf = -1*np.sin(-(90-theta))*xref_t + np.cos(-(90-theta))*yref_t

		if self.u_type == "Pure Pursuit":
			Ld_center = ((0-xref_tf)**2+(0-yref_tf)**2)**(0.5) 
			Ld = ((0-xref_tf)**2+(-L/2 - yref_tf)**2)**(0.5) 
			# above is same as L1 = ((x-xref)**2+(y-yref)**2)**(0.5)

		# error def and ops
		e_vel = v - vref
		gamma = 2*(0-xref_tf) / (Ld**2) # curvature in pure-pursuit

		self.e = np.array([[e_vel], [(0-xref_tf)]])
        	self.esum = self.esum + self.e*self.dt; self.eder = (self.e - eold) / self.dt

		# calculate control inputs	
		a = -Kp*self.e[0,0] +vref_dot - Ki*self.esum[0,0] - Kd*self.eder[0,0]
		delta = gamma #+ 2*Ki*self.esum[1,0] + Kd*self.eder[1,0]
		#print(xref_tf)

        	self.u = np.array([[a],[delta]])
        	self.yref_list.append(np.array([[xref],[yref],[theta_ref],[vref]])[:,0])


        return self.u 

    def generate_error(self):
	if self.NOW>=(self.s1-self.dt) and self.NOW<self.e1:
            err = np.array([[4],[4],[0],[0]])
        elif self.NOW>=(self.s2-self.dt) and self.NOW<self.e2:
            err = np.array([[-4],[-4],[0],[0]])
        else:
            err = np.array([[0],[0],[0],[0]])

	return err

    def do_error_analysis(self):
	return 0

    def plot_curves_car(self):

	for i in range(len(self.x)):
		plt.figure(i)
		reference = np.array(self.yref_list)
		#xaxis = np.reshape(np.linspace(0,self.T,self.T/self.dt + 1), (len(self.yref_list), 1))
		xaxis = np.reshape(np.array(self.NOW_list), (len(self.yref_list), 1))
		
		cdn0_a = (xaxis<=(self.e1-self.dt))
		cdn0_b = (xaxis>=(self.e1-self.dt)) & (xaxis<=(self.e2-self.dt)) 
		cdn0_c = (xaxis>=self.e2-self.dt)
		cdn0_a_big = np.append(cdn0_a, cdn0_a, axis=1)
		cdn0_b_big = np.append(cdn0_b, cdn0_b, axis=1) 
		cdn0_c_big = np.append(cdn0_c, cdn0_c, axis=1) 

		cdn1_a = (xaxis>=(self.s1-2*self.dt)) & (xaxis<=(self.e1))
		cdn1_b = (xaxis>=(self.s2-2*self.dt)) & (xaxis<=self.e2)
		cdn1_a_big = np.append(cdn1_a, cdn1_a, axis=1)
		cdn1_b_big = np.append(cdn1_b, cdn1_b, axis=1)
		for j in range(len(self.x)-2):
			cdn0_a_big = np.append(cdn0_a_big, cdn0_a, axis=1)
			cdn0_b_big = np.append(cdn0_b_big, cdn0_b, axis=1)
			cdn0_c_big = np.append(cdn0_c_big, cdn0_c, axis=1)

			cdn1_a_big = np.append(cdn1_a_big, cdn1_a, axis=1)
			cdn1_b_big = np.append(cdn1_b_big, cdn1_b, axis=1)
		self.actual = np.array(self.actual)
		self.actualy = np.array(self.actualy)
		self.measured = np.array(self.measured)
		self.estimated = np.array(self.estimated)
		pred_arr = np.reshape(np.array(self.predicted), (len(self.predicted), len(self.x)))
		
		if self.estimation_method == "none":
		    plt.plot(xaxis, reference[:,i], 'blue')
		    plt.plot(xaxis, self.actual[:,i], 'yellow')
		    plt.plot(xaxis, self.measured[:,i], 'green')
		    plt.plot(xaxis[cdn1_a], np.reshape(pred_arr[cdn1_a_big], ( sum(cdn1_a),len(self.x) ))[:,i], color='red')
		    plt.plot(xaxis[cdn1_b], np.reshape(pred_arr[cdn1_b_big], ( sum(cdn1_b),len(self.x) ))[:,i], color='red')
		elif self.estimation_method == "Kalman":
		    plt.plot(xaxis, reference[:,i], 'blue')
		    plt.plot(xaxis, self.actualy[:,i], color='grey')
		    plt.plot(xaxis, self.measured[:,i], 'green')
		    est_reshaped = np.reshape(self.estimated, (len(self.estimated), len(self.x)))
		    print(est_reshaped[np.where(cdn0_a_big)].shape, ( sum(cdn0_a),len(self.x) ) )
		    xEKF1 = np.reshape( est_reshaped[np.where(cdn0_a_big)], ( int(sum(cdn0_a)),len(self.x) ))
		    xEKF2 = np.reshape( est_reshaped[np.where(cdn0_b_big)], ( int(sum(cdn0_b)),len(self.x) ))
		    xEKF3 = np.reshape( est_reshaped[np.where(cdn0_c_big)], ( int(sum(cdn0_c)),len(self.x) ))
		    plt.plot(xaxis[cdn0_a], xEKF1[:,i], color='magenta', linestyle = '--')
		    plt.plot(xaxis[cdn1_a], np.reshape(pred_arr[cdn1_a_big], ( int(sum(cdn1_a)),len(self.x) ))[:,i], color='red')
		    plt.plot(xaxis[cdn0_b], xEKF2[:,i], color='magenta', linestyle = '--')
		    plt.plot(xaxis[cdn1_b], np.reshape(pred_arr[cdn1_b_big], ( int(sum(cdn1_b)),len(self.x) ))[:,i], color='red')
		    plt.plot(xaxis[cdn0_c], xEKF3[:,i], color='magenta', linestyle = '--')	
	
		if self.estimation_method == "Kalman":
		    plt.legend(['ref', 'GT', 'M', 'EKF', 'RF'], prop={"size":20})
		elif self.estimation_method == "none":
		    plt.legend(['ref', 'GT', 'M', 'RF'], prop={"size":20})
        	if i == 0: 
			plt.ylabel("X position", fontsize=20)
			plt.ylim([-15, 15])
		elif i == 1: 
			plt.ylabel("Y position", fontsize=20)
			plt.ylim([-15, 15])
		elif i == 2: plt.ylabel("Heading Angle", fontsize=20)
		elif i == 3: plt.ylabel("Velocity", fontsize=20)
		elif i == 4: plt.ylabel("Steering Angle", fontsize=20)
        	plt.xlabel("time step", fontsize=20)
		plt.grid()

	plt.show()

        
        plt.figure(4)
        plt.scatter(self.t_when_CkPt_used_DeadReck, self.t_of_CkPt_used_DeadReck, c='blue')
        plt.scatter(self.t_all_CkPts, self.t_all_CkPts, c='orange')
        plt.ylabel("time step of checkpoint used", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
	plt.show()
        
        plt.figure(5)
        plt.plot(reference[:,0], reference[:,1], 'blue')
        plt.plot(self.measured[:,0], self.measured[:,1], 'green')
	if self.estimation_method == "none":
		plt.plot(self.actual[:,0], self.actual[:,1], 'grey')
		plt.plot(np.reshape(pred_arr[cdn1_a_big], (sum(cdn1_a),len(self.x)))[:,0], np.reshape(pred_arr[cdn1_a_big], (sum(cdn1_a),len(self.x)))[:,1], 'red')
		plt.plot(np.reshape(pred_arr[cdn1_b_big], (sum(cdn1_b),len(self.x)))[:,0], np.reshape(pred_arr[cdn1_b_big], (sum(cdn1_b),len(self.x)))[:,1], 'red')
	elif self.estimation_method == "Kalman":
		plt.plot(self.actualy[:,0], self.actualy[:,1], 'grey')
		plt.plot(xEKF1[:,0], xEKF1[:,1], color='magenta', linestyle = '--')
		plt.plot(np.reshape(pred_arr[cdn1_a_big], (sum(cdn1_a),len(self.x)))[:,0], np.reshape(pred_arr[cdn1_a_big], (sum(cdn1_a),len(self.x)))[:,1], 'red')
		plt.plot(xEKF2[:,0], xEKF2[:,1], color='magenta', linestyle = '--')
		plt.plot(np.reshape(pred_arr[cdn1_b_big], (sum(cdn1_b),len(self.x)))[:,0], np.reshape(pred_arr[cdn1_b_big], (sum(cdn1_b),len(self.x)))[:,1], 'red')
		plt.plot(xEKF3[:,0], xEKF3[:,1], color='magenta', linestyle = '--')
        	
	
	if self.estimation_method == "Kalman":
		plt.legend(['ref', 'M', 'GT', 'EKF', 'RF'])
	elif self.estimation_method == "none":
		plt.legend(['ref', 'M', 'GT', 'RF'])

        plt.grid()
	plt.show()

	plt.figure(6)
	plt.plot(xaxis, self.dt_list)
	plt.ylabel("time step size", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
	plt.show()

	plt.figure(7)
	plt.plot(xaxis, self.CkPt_buffer_time_list, xaxis, self.RF_time_list, xaxis, self.other_time_list)
	plt.legend(['time to check and save checkpoint', 'time for RF', 'time for other'], prop={"size":14})
	plt.ylabel("time", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
        plt.show()


