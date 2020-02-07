import numpy as np
from scipy.linalg import expm
from matplotlib import pyplot as plt

class safe_auto_nonlinear(object):
    # Constructor
    def __init__(self, func_f, func_A, func_g, func_C, x0, u0, y0, interval, std, T, ADM, EM, u_type, devID, RFM="DeadReck"):
        self.NOW = 0; self.check = 0;  
        
        self.attk_detect_method = ADM; self.rollfwd_method = RFM; self.estimation_method = EM
        self.T = T; self.dt = 0; self.t_r = self.NOW
        self.x = x0; self.u = u0; self.y = y0; self.CkPt = np.array([x0,u0,y0]); 
        self.xe = x0
        
        self.devID = devID
        self.ustore = []; self.ystore = []; self.measured = []; self.actual = []; self.actualy = []; self.predicted = []; self.estimated = []   

        self.yref = 0; self.yref_list = []; self.func_f = func_f; self.func_A = func_A; self.func_g = func_g; self.func_C = func_C        
        self.P = np.eye(len(x0))
        self.std = std
        self.Q = self.std**2 * np.eye(len(x0)); self.R = self.std**2 * np.eye(len(y0))
        
        self.e = 0; self.eder = 0; self.esum = 0 
        self.e2 = 0; self.e2der = 0; self.e2sum = 0 
	self.time_to_CkPt_val = 0; self.NOW_old = 0
        # changed above --------------------------------------
        
        self.interval = interval; 
        self.t_when_CkPt_used_DeadReck = []; self.t_of_CkPt_used_DeadReck = []; self.t_all_CkPts = []
        self.t_when_CkPt_used_Kalman = []; self.t_of_CkPt_used_Kalman = [];
        self.edgeCkPt = []
        
        self.attk_ct = 0
        self.s1 = 3.5; self.e1 = 5; self.s2 = 8.5; self.e2 = 10
        self.u_type = u_type

	self.time_to_CkPt_val = 0; self.NOW_old = 0; self.dt_list = []; self.NOW_list = []	
	self.CkPt_buffer_time_list = []; self.RF_time_list = []; self.other_time_list = []
        
    def anomaly_detect(self):
        # attack detection algo goes here
        if self.attk_detect_method=="interval":
            # attack detection algo goes here
            if self.NOW>=self.s1 and self.NOW <self.e1:
                self.devID = np.array([[0],[0],[1]])
            elif self.NOW>=self.s2 and self.NOW <self.e2:
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
        
    def StartControl(self, tp): 
        
        # save value predicted after rollfwd
        self.predicted.append(self.y[:,0]) #takes first column of np.array
        
        # advance system in simulation
        eold = self.e
        e2old = self.e2
        self.Advance()

        # calculate & send PID control command to actuator
########### Change as appropriate for nonlinear control
        if self.u_type == "PID": #PID
            Kp = tp[0]; Kd = tp[1]; Ki = tp[2]
            xref = 2*np.cos(self.NOW); yref = 2*np.sin(self.NOW)
            xref_dot = -yref; yref_dot = xref
            
            phi_ref = np.arctan(yref_dot / xref_dot)
            self.e = phi_ref - self.y[2,0]
            #self.e = np.arctan(np.sin(phi_ref - self.y[2,0])/np.cos(phi_ref - self.y[2,0]))
            self.esum = self.esum + self.e*self.dt; self.eder = (self.e - eold) / self.dt
            w = Kp*self.e + Kd*self.eder + Ki*self.esum
            
            #self.e2 = xref - self.y[0,0]
            #self.e2sum = self.e2sum + self.e2*self.dt; self.e2der = (self.e2 - e2old) / self.dt
            #v = Kp*self.e2 + Kd*self.e2der + Ki*self.e2sum
            v = (xref_dot**2 + yref_dot**2)**(0.5)
            
            self.u = np.array([[v],[w]])
            self.yref_list.append(np.array([[xref],[yref],[phi_ref]])[:,0]) 
            
        elif self.u_type == "linear":
            k1 = tp[0]; k2 = tp[1]; l = tp[2]
            xref = 2*np.cos(self.NOW); yref = 2*np.sin(self.NOW)
            xref_dot = -yref; yref_dot = xref
            
            phi_ref = np.arctan( yref - self.x[1,0] / xref - self.x[0,0])
            phi = self.y[2,0];
            #phi = phi_ref
            x = self.y[0,0] + l*np.cos(phi); y = self.y[1,0] + l*np.sin(phi)
            v = np.cos(-phi)*( xref_dot + k1*(xref - x) ) - np.sin(-phi)*( yref_dot + k2*(yref - y) )
            w = (np.sin(-phi)*( xref_dot + k1*(xref - x) ) + np.cos(-phi)*( yref_dot+ k2*(yref - y) )) / l
            
            self.u = np.array([[v],[w]])
            self.yref_list.append(np.array([[xref],[yref],[phi_ref]])[:,0]) 
            
        elif self.u_type == "nonlinear":
            self.u = np.array([[0],[0]])
            
        #self.pub_to_actuator()
        return self.u
    
    def RollForward(self, x_prev):
        x = self.CkPt[0]
            
        if self.rollfwd_method == "DeadReck":
            self.t_of_CkPt_used_DeadReck.append(self.t_r)
            No = int((self.NOW-self.t_r)/self.dt)-1
            for i in range(No):
		#print(No, len(self.ustore), i)
                x = self.func_f(x, self.ustore[i], self.dt)
            self.t_when_CkPt_used_DeadReck.append(self.NOW)
            x[np.where(self.devID==1)] = x_prev[np.where(self.devID==1)]
            #x[np.where(self.devID==0)] = x[np.where(self.devID==0)]
        
        return x, self.func_g(x)
        
    def Advance(self):
        w = np.reshape(np.random.normal(0, self.std, len(self.x)), self.x.shape) #selftem noise
        v = np.reshape(np.random.normal(0, self.std, len(self.y)), self.y.shape) #sensor noise
        self.x = self.func_f(self.x, self.u, self.dt) + w
        self.y = self.func_g(self.x) + v       
        
    def time_to_CkPt(self):
        already_appended=0
        do_CkPt = 0
        # if checkpoint interval reached and no attack
        if self.NOW%self.interval==0 and self.check==0:
            do_CkPt = 1
            already_appended=1
            self.t_all_CkPts.append(self.NOW)            
            
        if already_appended==0:
            self.ustore.append(self.u); self.ystore.append(self.y)
        return do_CkPt


    def time_to_CkPt_coordinated(self):
        already_appended=0
        do_CkPt = 0
        # if checkpoint time command recieved and no attack
        if self.time_to_CkPt_val==1 and self.check==0:
            do_CkPt = 1
            already_appended=1
            self.t_all_CkPts.append(self.NOW)            
            
        if already_appended==0:
            self.ustore.append(self.u); self.ystore.append(self.y)
        return do_CkPt
    
    def create_CkPt(self):
        self.CkPt = np.array([self.x, self.u, self.y])
        self.t_r = self.NOW 
        self.ustore = [self.u]; self.ystore = [self.y] #remove all u,u before checkpoint

    def get_x_estimate_from_sensors(self):
        self.actualy.append(self.y[:,0])        
        # attack (measured = actual + attk)      
        if self.NOW>=(self.s1-self.dt) and self.NOW<self.e1:
            err = np.array([[5],[5],[0.1]])
        elif self.NOW>=(self.s2-self.dt) and self.NOW<self.e2:
            err = np.array([[-5],[-5],[-0.1]])
        else:
            err = 0
        self.y_m = self.y + err
        self.measured.append(self.y_m[:,0])
        
########### (E)KF implemented for (non)linear system below        
        if self.estimation_method == "Kalman":
            if self.check==0:    
                # if an attack has just finished & currently not under attack, start new KF from new rollfwd data predicted checkpoint
                if self.attk_ct>0 and self.change == -1:            
                    self.xe = self.CkPt[0]
            # predict
            self.xe = self.func_f(self.xe, self.u, self.dt)
            A = self.func_A(self.xe, self.u)
            A_d = expm(A*self.dt)
            self.P = np.matmul(np.matmul(A_d, self.P), A_d.T) + self.Q
            # kalman gain
            C = self.func_C(self.xe)
            L = np.matmul(self.P, np.matmul(C.T, np.linalg.inv(    np.matmul(np.matmul(C, self.P), C.T) + self.R)))        
            # update
            self.xe = self.xe + np.matmul(L, (self.y_m - self.func_g(self.xe)))
            self.P = np.matmul((np.identity(len(self.P)) - np.matmul(L, C)), self.P)
            # save
            self.ye = self.func_g(self.xe)
            self.estimated.append(self.ye[:,0])
        
            return self.xe
            
        elif self.estimation_method == "none":
            self.actual.append(self.x[:,0])
            self.x = self.x + err
            return self.x
                  
    def pub_to_actuator(self):
        return 0
        
    def plot_curves(self):
        plt.figure(10)
        reference = np.reshape(np.array(self.yref_list), (len(self.yref_list),1))
        #xaxis = np.reshape(np.linspace(0,self.T,self.T/self.dt + 1), reference.shape)
	xaxis = np.reshape(np.array(self.NOW_list), reference.shape)
        
        cdn0_a = (xaxis<=(self.e1-self.dt)) 
        cdn0_b = (xaxis>=(self.e1)) & (xaxis<=(self.e2-self.dt)) 
        cdn0_c = (xaxis>=self.e2)
        
        if self.estimation_method == "none":
            plt.plot(xaxis, reference, 'blue', xaxis, self.actual, 'yellow', xaxis, self.measured, 'green')
        elif self.estimation_method == "Kalman":
            plt.plot(xaxis, reference, 'blue')
            plt.plot(xaxis, self.actualy, color='grey')
            plt.plot(xaxis, self.measured, 'green')
            x_reshaped = np.reshape(xaxis, (len(xaxis),1)); est_reshaped = np.reshape(self.estimated, (len(self.estimated), 1))
            plt.plot(x_reshaped[np.where(cdn0_a)], est_reshaped[np.where(cdn0_a)], color='purple', linestyle = '--')
            plt.plot(x_reshaped[np.where(cdn0_b)], est_reshaped[np.where(cdn0_b)], color='magenta', linestyle = '--')
            plt.plot(x_reshaped[np.where(cdn0_c)], est_reshaped[np.where(cdn0_c)], color='violet', linestyle = '--')                    
        cdn1_a = (xaxis>=(self.s1-2*self.dt)) & (xaxis<=(self.e1)) 
        cdn1_b = (xaxis>=(self.s2-2*self.dt)) & (xaxis<=self.e2)
        pred_arr = np.reshape(np.array(self.predicted), (len(self.predicted), 1))
        plt.plot(xaxis[cdn1_a], pred_arr[cdn1_a], color='red')
        plt.plot(xaxis[cdn1_b], pred_arr[cdn1_b], color='red')
        
        if self.estimation_method == "Kalman":
            plt.legend(['ref', 'True Measurement = y', 'Measurement = y + attk', 'EKF1', 'EKF2', 'EKF3', 'RollFwd Prediction = x_hat (CkPt)[1]'])
        elif self.estimation_method == "none":
            plt.legend(['ref', 'Ground Truth (y = x[1])', 'Measured (y + a)', 'RollFwd Prediction = x_hat (CkPt)[1]'])
        plt.grid()
        plt.show()
        
        plt.figure(11)
        plt.scatter(self.t_when_CkPt_used_DeadReck, self.t_of_CkPt_used_DeadReck, c='blue')
        plt.scatter(self.t_all_CkPts, self.t_all_CkPts, c='orange')
        plt.ylabel("Time of Checkpoint used")
        plt.xlabel("Time when Checkpoint used for prediction")
        plt.grid()
        plt.show()
        
        
        
        
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
		
		if self.estimation_method == "none":
		    plt.plot(xaxis, reference[:,i], 'blue')
		    plt.plot(xaxis, self.actual[:,i], 'yellow')
		    plt.plot(xaxis, self.measured[:,i], 'green')
		elif self.estimation_method == "Kalman":
		    plt.plot(xaxis, reference[:,i], 'blue')
		    plt.plot(xaxis, self.actualy[:,i], color='grey')
		    plt.plot(xaxis, self.measured[:,i], 'green')
		    est_reshaped = np.reshape(self.estimated, (len(self.estimated), len(self.x)))

		    xEKF1 = np.reshape( est_reshaped[np.where(cdn0_a_big)], ( sum(cdn0_a),len(self.x) ))
		    xEKF2 = np.reshape( est_reshaped[np.where(cdn0_b_big)], ( sum(cdn0_b),len(self.x) ))
		    xEKF3 = np.reshape( est_reshaped[np.where(cdn0_c_big)], ( sum(cdn0_c),len(self.x) ))

		    plt.plot(xaxis[cdn0_a], xEKF1[:,i], color='purple', linestyle = '--')
		    plt.plot(xaxis[cdn0_b], xEKF2[:,i], color='magenta', linestyle = '--')
		    plt.plot(xaxis[cdn0_c], xEKF3[:,i], color='violet', linestyle = '--')
		cdn1_a = (xaxis>=(self.s1-2*self.dt)) & (xaxis<=(self.e1))
		cdn1_a_big = np.append( np.append(cdn1_a, cdn1_a, axis=1), cdn1_a, axis=1)
		cdn1_b = (xaxis>=(self.s2-2*self.dt)) & (xaxis<=self.e2)
		cdn1_b_big = np.append( np.append(cdn1_b, cdn1_b, axis=1), cdn1_b, axis=1)
		pred_arr = np.reshape(np.array(self.predicted), (len(self.predicted), len(self.x)))
		
		plt.plot(xaxis[cdn1_a], np.reshape(pred_arr[cdn1_a_big], ( sum(cdn1_a),len(self.x) ))[:,i], color='red')
		plt.plot(xaxis[cdn1_b], np.reshape(pred_arr[cdn1_b_big], ( sum(cdn1_b),len(self.x) ))[:,i], color='red')
		
		if self.estimation_method == "Kalman":
		    plt.legend(['ref', 'GT', 'M', 'EKF1', 'EKF2', 'EKF3', 'RF'], prop={"size":20})
		elif self.estimation_method == "none":
		    plt.legend(['ref', 'GT', 'M', 'RF'], prop={"size":20})
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
        plt.grid()
	plt.show()

	plt.figure(7)
	plt.plot(xaxis, self.CkPt_buffer_time_list, xaxis, self.RF_time_list, xaxis, self.other_time_list)
	plt.legend(['time to load checkpoint', 'time for RF', 'time for other'], prop={"size":20})
	plt.ylabel("time", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
        plt.show()













