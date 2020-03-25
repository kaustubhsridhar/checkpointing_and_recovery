import numpy as np
import rospy
from scipy.linalg import expm
from matplotlib import pyplot as plt

class timer_start():
    def __init__(self):
        self.t_start = rospy.get_time()
    def elapsed(self):
        return rospy.get_time() - self.t_start

class safe_auto_nonlinear(object):
    # Constructor
    def __init__(self, func_f, func_A, func_g, func_C, x0, u0, y0, interval, T, EM, u_type, devID, Q, R):
        self.NOW = 0; self.check = 0;  
        
        self.estimation_method = EM
        self.T = T; self.dt = 0; self.t_r = self.NOW
        self.x = x0; self.u = u0; self.y = y0; self.CkPt = np.array([x0,u0,y0]); 
        self.xe = x0
        
        self.devID = devID
        self.ustore = []; self.ystore = []; self.measured = []; self.actual = []; self.actualy = []; self.predicted = []; self.estimated = []   

        self.yref = 0; self.yref_list = []; self.func_f = func_f; self.func_A = func_A; self.func_g = func_g; self.func_C = func_C
        self.P = np.eye(len(x0))
        self.Q = Q; self.R = R
        
        self.ydot = 0; self.ydsum = 0
        self.interval = interval; 
        self.t_when_CkPt_used_DeadReck = []; self.t_of_CkPt_used_DeadReck = []; self.t_all_CkPts = []
        self.t_when_CkPt_used_Kalman = []; self.t_of_CkPt_used_Kalman = [];
        self.edgeCkPt = []
        
        self.attk_ct = 0
        self.s1 = 3.5; self.e1 = 5; self.s2 = 8.5; self.e2 = 10
        self.u_type = u_type

	self.time_to_CkPt_val = 0; self.NOW_old = 0; self.dt_list = []; self.NOW_list = []
	self.CkPt_buffer_time_list = []; self.RF_time_list = []; self.other_time_list = []
	self.rf_count = 0
        
    def anomaly_detect(self):
        # attack detection algo goes here
        if self.NOW>=self.s1 and self.NOW <self.e1:
            self.devID = np.array([[1],[0]])
        elif self.NOW>=self.s2 and self.NOW <self.e2:
            self.devID = np.array([[1],[0]])
        else:
            self.devID = np.array([[1],[1]])

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
        
    def StartControl(self, w_d): 
        
        # save value predicted after rollfwd
        self.predicted.append(self.y[:,0]) #takes first column of np.array
        
        # advance system in simulation
        selfyold = self.y
        self.Advance()

        # PID for DC motor with desired value w_d
        if self.u_type == "PID": #PID
            Kp = 6.6/0.5; Kd = 1.1/4; Ki = 6.1/4
            self.yref = np.array([[w_d[0,0]]])
            self.ydot = (self.y-selfyold)/self.dt; self.ydsum = self.ydsum + (self.yref - self.y)*self.dt
            self.u = Kp*(self.yref - self.y) - Kd*self.ydot + Ki*self.ydsum
        elif self.u_type == "nonlinear":
            self.u = np.array([[0]])
            
        self.yref_list.append(self.yref) 
        self.pub_to_actuator()
        return self.u
    
    def RollForward(self, x_prev):
	# if this is the first roll-forward, roll-forward from the last checkpoint
	self.t_of_CkPt_used_DeadReck.append(self.t_r)
	if self.prev_check == 0 and self.check == 1:
		x = self.CkPt[0]
		No = int((self.NOW-self.t_r)/self.dt)-1
		for i in range(len(self.ustore)):
		    x = self.func_f(x, self.ustore[i], self.dt)
	# if this is second+ roll-forward, just rf from previous roll-forward predicted val	
	else:
		x = self.func_f(self.x, self.ustore[-1], self.dt)
	
	self.t_when_CkPt_used_DeadReck.append(self.NOW)
	x[np.where(self.devID==1)] = x_prev[np.where(self.devID==1)]
	#x[np.where(self.devID==0)] = x[np.where(self.devID==0)]
        
        return x, self.func_g(x)

    def RollForward_slow(self, x_prev):
        x = self.CkPt[0]
            
        self.t_of_CkPt_used_DeadReck.append(self.t_r)
        No = int((self.NOW-self.t_r)/self.dt)-1
        for i in range(len(self.ustore)):
		#print(No, len(self.ustore), i)
        	x = self.func_f(x, self.ustore[i], self.dt)
        self.t_when_CkPt_used_DeadReck.append(self.NOW)
        x[np.where(self.devID==1)] = x_prev[np.where(self.devID==1)]
        #x[np.where(self.devID==0)] = x[np.where(self.devID==0)]
        
        return x, self.func_g(x)
        
    def Advance(self):
        w = np.transpose( np.random.multivariate_normal(np.zeros(len(self.x)), self.Q, (1)) ) #process noise
        v = np.transpose( np.random.multivariate_normal(np.zeros(len(self.y)), self.R, (1)) ) #sensor noise
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

    def generate_error(self):
	if self.NOW>=(self.s1-self.dt) and self.NOW<self.e1:
            err = 20000
        elif self.NOW>=(self.s2-self.dt) and self.NOW<self.e2:
            err = -20000
        else:
            err = 0
	return err

    def get_x_estimate_from_sensors(self):
        self.actualy.append(self.y[:,0])        
        # attack (measured = actual + attk)
	err = self.generate_error()
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
            self.xe = self.xe + np.matmul(L, (self.y_m - self.func_g(self.xe, self.u)))
            self.P = np.matmul((np.identity(len(self.P)) - np.matmul(L, C)), self.P)
            # save
            self.ye = self.func_g(self.xe, self.u)
            self.estimated.append(self.ye[:,0])
        
            return self.xe
            
        elif self.estimation_method == "none":
            self.actual.append(self.x[1,0])
            self.x[1] = self.x[1] + err
            return self.x
                  
    def pub_to_actuator(self):
        return 0
        
    def plot_curves(self):
        plt.figure(10)
        reference = np.reshape(np.array(self.yref_list), (len(self.yref_list),1))
        xaxis = np.reshape(np.array(self.NOW_list), reference.shape)
        
        cdn0_a = (xaxis<=(self.e1-self.dt)) 
        cdn0_b = (xaxis>=(self.e1)) & (xaxis<=(self.e2-self.dt)) 
        cdn0_c = (xaxis>=self.e2)
        
        if self.estimation_method == "none":
            plt.plot(xaxis, reference, 'blue')
	    plt.plot(xaxis, self.actual, 'yellow', xaxis, self.measured, 'green')
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
	    plt.legend(['ref', 'GT', 'M', 'EKF1', 'EKF2', 'EKF3', 'RF'], prop={"size":20})
        elif self.estimation_method == "none":
            plt.legend(['ref', 'Ground Truth (y = x[1])', 'Measured (y + a)', 'RollFwd Prediction = x_hat (CkPt)[1]'])
	plt.ylabel("angular velocity", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
	plt.show()
        
        plt.figure(11)
        plt.scatter(self.t_when_CkPt_used_DeadReck, self.t_of_CkPt_used_DeadReck, c='blue')
        plt.scatter(self.t_all_CkPts, self.t_all_CkPts, c='orange')
        plt.ylabel("time step of checkpoint used", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
	plt.show()

	plt.figure(12)
	plt.plot(xaxis, self.dt_list)
	plt.ylabel("time step size", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
	plt.show()

	plt.figure(14)
	plt.plot(xaxis, self.CkPt_buffer_time_list, xaxis, self.RF_time_list, xaxis, self.other_time_list)
	plt.legend(['time to check and save checkpoint', 'time for RF', 'time for other'], prop={"size":20})
	plt.ylabel("time", fontsize=20)
        plt.xlabel("time step", fontsize=20)
        plt.grid()
        plt.show()









