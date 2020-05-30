import numpy as np
from scipy.linalg import expm
from matplotlib import pyplot as plt
# ddrive class (here) inherits from base class
from safe_auto_nonlinear_base import safe_auto_nonlinear 

class safe_auto_nonlinear_ddrive(safe_auto_nonlinear):
    def __init__(self, func_f, func_A, func_g, func_C, x0, u0, y0, interval, T, EM, u_type, devID, Q, R, EA):
	super(safe_auto_nonlinear_ddrive, self).__init__(func_f, func_A, func_g, func_C, x0, u0, y0, interval, T, EM, u_type, devID, Q, R, EA) # brings everything over from parent class
	# add new variables for this child class below
	self.time_to_CkPt_val = 0; self.NOW_old = 0
        

    def anomaly_detect(self):
        # attack detection algo goes here
        if self.NOW>=self.s1 and self.NOW <=self.e1:
            self.devID = np.array([[0],[0],[1]])
        elif self.NOW>=self.s2 and self.NOW <=self.e2:
            self.devID = np.array([[0],[0],[1]])
        else:
            self.devID = np.array([[1],[1],[1]])

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
        
    def StartControl(self): 
        
        # save value predicted after rollfwd
        self.predicted.append(self.xe[:,0]) # converts col np.array into list
        
        # take old values
        eold = self.e

	# current states
	x = self.xe[0,0]; y = self.xe[1,0]; phi = self.xe[2,0]

        # PID
        if self.u_type == "PID": #PID
            Kp = 6.6/0.5; Kd = 1.1/4; Ki = 6.1/4
            xref = 2*np.cos(self.NOW); yref = 2*np.sin(self.NOW)
            xref_dot = -yref; yref_dot = xref
            phi_ref = np.arctan(yref_dot / xref_dot)

            self.e = phi_ref - phi
            self.esum = self.esum + self.e*self.dt; self.eder = (self.e - eold) / self.dt
            w = Kp*self.e + Kd*self.eder + Ki*self.esum
            
            v = (xref_dot**2 + yref_dot**2)**(0.5)
            
            self.u = np.array([[v],[w]])
            self.yref_list.append(np.array([[xref],[yref],[phi_ref]])[:,0]) 
        
	# dynamic inversion controller for diff drive mobile robot
        elif self.u_type == "linear":
            k1 = 6.6/2; k2 = 6.6/1.5; l = 0.01;
            xref = 2*np.cos(self.NOW); yref = 2*np.sin(self.NOW)
            xref_dot = -yref; yref_dot = xref
            phi_ref = np.arctan( yref - self.xe[1,0] / xref - self.xe[0,0])

            x = x + l*np.cos(phi); y = y + l*np.sin(phi)
            v = np.cos(-phi)*( xref_dot + k1*(xref - x) ) - np.sin(-phi)*( yref_dot + k2*(yref - y) )
            w = (np.sin(-phi)*( xref_dot + k1*(xref - x) ) + np.cos(-phi)*( yref_dot+ k2*(yref - y) )) / l
            
            self.u = np.array([[v],[w]])
            self.yref_list.append(np.array([[xref],[yref],[phi_ref]])[:,0]) 
            
        elif self.u_type == "nonlinear":
            self.u = np.array([[0],[0]])
            
        #self.pub_to_actuator()
        return self.u 

    def generate_error(self):
	if self.NOW>=(self.s1-self.dt) and self.NOW<self.e1:
            err = np.array([[5],[5],[0.1]])
        elif self.NOW>=(self.s2-self.dt) and self.NOW<self.e2:
            err = np.array([[-5],[-5],[-0.1]])
        else:
            err = 0

	return err

    def do_error_analysis(self):
	# get/define LTI system, system bounds info		
	A = np.array([ [0,0,1], [0,0,6], [0,0,0] ])
	epsilon_delta = np.array([[5],[5],[5]]); epsilon_omega = epsilon_delta; epsilon_gamma = np.array([[0.025],[0.025],[0.025]]) #since omega_k = gamma_k = N(0,0.01)
	bold_y = np.array([[6],[6],[0]]); Kj = 1*np.eye(3) # since err = + or - [[5],[5],[0]]
	k_1 = int(self.t_r); k = int(self.NOW)

	# errors, bounds calculated and saved for analysis
	if (self.NOW==self.s1-self.dt) or (self.NOW== self.s2-self.dt):
		self.det_time_err_list.append(self.x[:,0]-self.xe_o[:,0])
		self.edge_times.append(self.NOW)
		
		summed_term = np.array([[0.0],[0.0],[0.0]])
		for l in range(k_1, k+1):
			summed_term += np.matmul( np.abs( np.linalg.matrix_power(A, k - l) ), epsilon_omega)
		det_time_err_bound = np.matmul( np.abs( np.linalg.matrix_power(A, k - k_1) ), epsilon_delta) + summed_term + np.matmul( Kj, epsilon_gamma + bold_y)
		self.dte_bound_list.append(det_time_err_bound)

	if (self.NOW>self.s1-self.dt and self.NOW<self.e1) or (self.NOW>self.s2-self.dt and self.NOW<self.e2):
		self.pred_err_list.append(self.x[:,0]-self.xe[:,0])
		self.esti_err_list.append(self.x[:,0]-self.xe_o[:,0])
		self.attk_times.append(self.NOW)

		summed_term = np.array([[0.0],[0.0],[0.0]])
		for l in range(k_1, k+1):
			summed_term += np.matmul( np.abs( np.linalg.matrix_power(A, k - l) ), epsilon_omega)
		pred_err_bound = np.matmul( np.abs( np.linalg.matrix_power(A, k - k_1) ), epsilon_delta) + summed_term 
		esti_err_bound = epsilon_delta
		self.pe_bound_list.append(pred_err_bound)
		self.ee_bound_list.append(esti_err_bound)

    def plot_curves_car(self):

	for i in range(len(self.x)):
		plt.figure(i)
		reference = np.array(self.yref_list)
		xaxis = np.reshape(np.linspace(0,self.T,self.T/self.dt + 1), (len(self.yref_list), 1))
		#xaxis = np.reshape(np.array(self.NOW_list), (len(self.yref_list), 1))
		
		cdn0_a = (xaxis<=(self.e1-self.dt)) 
		cdn0_a_big = np.append( np.append(cdn0_a, cdn0_a, axis=1), cdn0_a, axis=1)
		cdn0_b = (xaxis>=(self.e1)) & (xaxis<=(self.e2-self.dt)) 
		cdn0_b_big = np.append( np.append(cdn0_b, cdn0_b, axis=1), cdn0_b, axis=1)
		cdn0_c = (xaxis>=self.e2)
		cdn0_c_big = np.append( np.append(cdn0_c, cdn0_c, axis=1), cdn0_c, axis=1)
		
		self.actual = np.array(self.actual)
		self.actualy = np.array(self.actualy)
		self.measured = np.array(self.measured)
		self.estimated = np.array(self.estimated)
		cdn1_a = (xaxis>=(self.s1-2*self.dt)) & (xaxis<=(self.e1))
		cdn1_a_big = np.append( np.append(cdn1_a, cdn1_a, axis=1), cdn1_a, axis=1)
		cdn1_b = (xaxis>=(self.s2-2*self.dt)) & (xaxis<=self.e2)
		cdn1_b_big = np.append( np.append(cdn1_b, cdn1_b, axis=1), cdn1_b, axis=1)
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

		    xEKF1 = np.reshape( est_reshaped[np.where(cdn0_a_big)], ( int(sum(cdn0_a)),len(self.x) ))
		    xEKF2 = np.reshape( est_reshaped[np.where(cdn0_b_big)], ( int(sum(cdn0_b)),len(self.x) ))
		    xEKF3 = np.reshape( est_reshaped[np.where(cdn0_c_big)], ( int(sum(cdn0_c)),len(self.x) ))

		    plt.plot(xaxis[cdn0_a], xEKF1[:,i], color='purple', linestyle = '--')
		    plt.plot(xaxis[cdn1_a], np.reshape(pred_arr[cdn1_a_big], ( int(sum(cdn1_a)),len(self.x) ))[:,i], color='red')
		    plt.plot(xaxis[cdn0_b], xEKF2[:,i], color='purple', linestyle = '--')
		    plt.plot(xaxis[cdn1_b], np.reshape(pred_arr[cdn1_b_big], ( int(sum(cdn1_b)),len(self.x) ))[:,i], color='red')
		    plt.plot(xaxis[cdn0_c], xEKF3[:,i], color='purple', linestyle = '--')
		
		if self.estimation_method == "Kalman":
		    plt.legend(['ref', 'GT', 'M', 'EKF', 'RF'], prop={"size":14})
		elif self.estimation_method == "none":
		    plt.legend(['ref', 'GT', 'M', 'RF'], prop={"size":14})
        	if i == 0: plt.ylabel("X position", fontsize=20)
		if i == 1: plt.ylabel("Y position", fontsize=20)
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
	elif self.estimation_method == "Kalman":
		plt.plot(self.actualy[:,0], self.actualy[:,1], 'grey')
		plt.plot(xEKF1[:,0], xEKF1[:,1], color='purple', linestyle = '--')
		plt.plot(xEKF2[:,0], xEKF2[:,1], color='magenta', linestyle = '--')
		plt.plot(xEKF3[:,0], xEKF3[:,1], color='violet', linestyle = '--')
	plt.plot(np.reshape(pred_arr[cdn1_a_big], (sum(cdn1_a),len(self.x)))[:,0], np.reshape(pred_arr[cdn1_a_big], (sum(cdn1_a),len(self.x)))[:,1], 'red')
	plt.plot(np.reshape(pred_arr[cdn1_b_big], (sum(cdn1_b),len(self.x)))[:,0], np.reshape(pred_arr[cdn1_b_big], (sum(cdn1_b),len(self.x)))[:,1], 'red')
        	
	
	if self.estimation_method == "Kalman":
		plt.legend(['ref', 'M', 'GT', 'EKF1', 'EKF2', 'EKF3', 'RF'])
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

	if self.error_analysis==1:
		
		xaxis_det_time = np.reshape(np.array(self.edge_times), (len(self.det_time_err_list), 1))
		xaxis_attk_time = np.reshape(np.array(self.attk_times), (len(self.pred_err_list), 1))

		dte_arr = np.reshape(self.det_time_err_list, (len(self.det_time_err_list), len(self.x)))
		pe_arr = np.abs(np.reshape(self.pred_err_list, (len(self.pred_err_list), len(self.x))))
		ee_arr = np.abs(np.reshape(self.esti_err_list, (len(self.esti_err_list), len(self.x))))

		dte_b_arr = np.reshape(self.dte_bound_list, (len(self.dte_bound_list), len(self.x)))
		pe_b_arr = np.reshape(self.pe_bound_list, (len(self.pe_bound_list), len(self.x)))
		ee_b_arr = np.reshape(self.ee_bound_list, (len(self.ee_bound_list), len(self.x)))

		plt.figure(8)
		#plt.plot(xaxis_det_time, dte_arr[:,0], '--ro')
		#plt.plot(xaxis_det_time, dte_arr[:,1], '-.ro')
		plt.plot(xaxis_attk_time, pe_arr[:,0], 'bo')
		plt.plot(xaxis_attk_time, ee_arr[:,0], 'b*')
		#plt.plot(xaxis_attk_time, pe_arr[:,1], '-.bo')
		plt.plot(xaxis_attk_time, ee_arr[:,2], 'go')
		#plt.plot(xaxis_det_time, dte_b_arr[:,0], '-r')
		#plt.plot(xaxis_det_time, dte_b_arr[:,1], ':r')
		plt.plot(xaxis_attk_time, pe_b_arr[:,0], '-b')
		#plt.plot(xaxis_attk_time, pe_b_arr[:,1], ':b')
		plt.plot(xaxis_attk_time, ee_b_arr[:,2], '-g')
		#plt.plot(xaxis_det_time, -1.0*dte_b_arr[:,0], '-r')
		#plt.plot(xaxis_det_time, -1.0*dte_b_arr[:,1], ':r')
		#plt.plot(xaxis_attk_time, -1.0*pe_b_arr[:,0], '-b')
		#plt.plot(xaxis_attk_time, -1.0*pe_b_arr[:,1], ':b')
		#plt.plot(xaxis_attk_time, -1.0*ee_b_arr[:,2], '-g')
		#plt.legend(['PE', 'EE', 'PE bound', 'EE bound'], prop={"size":14})
		plt.ylabel("errors", fontsize=20)
		plt.xlabel("time step", fontsize=20)
		plt.grid()
		plt.show()



