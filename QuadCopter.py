"""
The code is designed to use with Raspberry Pi, Pixhawk (via dronekit) and Arduino Leonardo
Creator:            Mana Saedan
Date Last Edited:   23 June 2017
Development Logs:
    - Making two classes to handle Lidar and HardOverride
    - Seperate thread to 
        - reading sensor (via dronekit) 
        - sensing control command (via arduino)
        - height control
        - reading range data
        - Detect obstacle and avoid base on potential field method
"""
import math
import time
import sys
import serial
import thread
from dronekit import connect, VehicleMode, LocationGlobalRelative
import serial.tools.list_ports

from threading import Thread, Lock
from sys import stdout

from HardOverride import HardOverride
from Lidar import Lidar

import tty, termios


"""
    Aircraft RC channel mapping
    Channel 1 = Roll
    Channel 2 = Pitch
    Channel 3 = Throttle
    Channel 4 = Yaw
    Channel 5 = Flight Mode
"""

class QuadCopter():
    def __init__(self):
        #Record of RC value
        self.RC_Ch1 = 1500
        self.RC_Ch2 = 1500
        self.RC_Ch3 = 1000
        self.RC_Ch4 = 1500
        self.RC_Ch5 = 1000

        #Override RC value
        self.RC_Override1 = 1500
        self.RC_Override2 = 1500
        self.RC_Override3 = 1500
        self.RC_Override4 = 1500

        self.Loop = 0

        #Parameter of the aircraft
        self.Aircraft_Armed=0
        self.Aircraft_Height =0
        self.Aircraft_Velocity=[0 for i in range(3)]
        self.AirCraft_Battery=0
        
        self.errH = 0.0
        self.errV = 0.0
    
        self.hardOverride   = None
        self.vehicle        = None
        self.lidar          = None


################## Function to search for PX4 FMU connect with RPi ###################
    def FindPixhawk(self):
        ListPortInfo = serial.tools.list_ports.comports()
        for list in ListPortInfo:
            if (list.product != None):
                if (list.product.find("PX4") != -1):
                    return list.device
        return None

################################ Change RC value ###################################
    def ChangeRCVal(self, Steps, Delay, Ch1, Ch2, Ch3, Ch4, Ch5):
        #Calculate rate of change for RC values
        ch1_step = (Ch1 - self.RC_Ch1)/Steps
        ch2_step = (Ch2 - self.RC_Ch2)/Steps
        ch3_step = (Ch3 - self.RC_Ch3)/Steps
        ch4_step = (Ch4 - self.RC_Ch4)/Steps
        ch5_step = (Ch5 - self.RC_Ch5)/Steps
    
        for i in range(1,Steps):
            self.RC_Ch1 = self.RC_Ch1 +ch1_step
            self.RC_Ch2 = self.RC_Ch2 +ch2_step
            self.RC_Ch3 = self.RC_Ch3 +ch3_step
            self.RC_Ch4 = self.RC_Ch4 +ch4_step
            self.RC_Ch5 = self.RC_Ch5 +ch5_step

            self.hardOverride.Send(self.RC_Ch1, self.RC_Ch2, self.RC_Ch3, self.RC_Ch4, self.RC_Ch5)
            time.sleep(Delay)
    
        #Set final RC value to desire value
        self.RC_Ch1 = Ch1
        self.RC_Ch2 = Ch2
        self.RC_Ch3 = Ch3
        self.RC_Ch4 = Ch4
        self.RC_Ch5 = Ch5
        self.hardOverride.Send(self.RC_Ch1, self.RC_Ch2, self.RC_Ch3, self.RC_Ch4, self.RC_Ch5)
        return

################## Check whether aircraft is armed or not armed ####################
    def CheckAircraftArm(self, IsCheckArm):
        #Check whether now the aircraft state
        wait_count=0
        while (wait_count<5):
            #Check if vehicle is armed
            if (IsCheckArm):
                if self.vehicle.armed:
                    return False
            #Check if vehicle is disarmed
            else:
                if not self.vehicle.armed:
                    return False
            time.sleep(1)
            wait_count = wait_count+1
        return True

############################# Check Aircraft Arming ############################
    def isArm(self):
        return self.vehicle.armed

############################# Aircraft arming sequence ############################
    def ArmAircraftLoiter(self, Step, WaitDelay):
        #Override RC channels
        print "Set flight mode to stabilize."
        self.vehicle.mode = VehicleMode('STABILIZE')
        self.ChangeRCVal(Step, WaitDelay, 1500, 1500, 1000, 1500, 1000)

        #Check whether the flight mode now is in "STABILIZE"
        time.sleep(1)
        if (self.vehicle.mode != VehicleMode('STABILIZE')):
            print "ERROR: Aircraft is not in stabilized mode"
            return False
        
        #Arm in 'STABLIZE' mode first. Make Ch3 (Throttle) low and Ch4 (Yaw) high.
        print "Arm aircraft in stabilize mode."
        self.vehicle.armed = True
        self.ChangeRCVal(Step, WaitDelay, 1500, 1500, 1000, 2000, 1000)
        
        
        if self.CheckAircraftArm(True):
            return False

        #Then disarm motor. Make Ch3 (Throttle) low and Ch4 (Yaw) low.
        print "Disarm aircraft."
        self.vehicle.armed = False
        self.ChangeRCVal(Step, WaitDelay, 1500, 1500, 1000, 1000, 1000)
            
        #Quit the program if aircraft is still armed
        if self.CheckAircraftArm(False):
            return False

        #Set mode to loiter
        print "Set flight mode to loiter."
        self.vehicle.mode = VehicleMode('LOITER')
        self.ChangeRCVal(Step, WaitDelay, 1500, 1500, 1000, 1000, 2000)

        #Check whether the flight mode now is in "LOITER"
        time.sleep(1)
        if (self.vehicle.mode != VehicleMode('LOITER')):
            print "ERROR: Aircraft is not in loiter mode"
            return False

        #Try to arm aircraft in 'LOITER MODE'
        print "Try to arm aircraft in loiter mode."
        self.vehicle.armed = True
        self.ChangeRCVal(Step, WaitDelay, 1500, 1500, 1000, 2000, 2000)
            
        #Quit the program if aircraft is armed
        if self.CheckAircraftArm(True):
            return False

        #Return RC "stick" to neutral position
        print "Aircraft is armed loiter mode."
        self.ChangeRCVal(Step, WaitDelay, 1500, 1500, 1000, 1500, 2000)
        return True

    ############################## RC H/W Override Loop ########################
    def RCOverideLoop(self):
        print "RC override loop starting...\r"
        #Excecute this loop 50 Hz
        while (self.Loop):
            #self.ChangeRCVal(5, 0.008, self.RC_Override1, self.RC_Override2, self.RC_Override3, self.RC_Override4, 2000)
            self.ChangeRCVal(1, 0.06, self.RC_Override1, self.RC_Override2, self.RC_Override3, self.RC_Override4, 2000)
            time.sleep(0.06)
        
        print "RC override loop terminated!\r"
        #When override loop is terminate, land the aircraft
        self.ChangeRCVal(10, 0.03, 1500, 1500, 1500, 1500, 2000)
        return


    ############################### LIDAR Reading Loop ##########################
    def LidarReadingLoop(self):
        print "Lidar reading loop starting...\r"
        index = 1
        oldindex=1;
        while(self.Loop):
            self.mymutex.acquire()
            #Read lidar range and digitize the data
            index = self.lidar.Read(10.0,10.0)
            self.mymutex.release()
            
            if (index<oldindex):
                #Create thread to read range from MyLidar
                try:
                    self.mapThread = Thread(target=self.ObstacleAvoidance)
                    self.mapThread.start()
                except:
                    print "Error: Unable to start thread to read MyLidar"
            
            oldindex=index

        print "Lidar reading loop terminated!\r"
        return

    ######################### Obstacle avoidance loop #######################
    #Thread to construct grid map
    def ObstacleAvoidance(self):
        #Check obstacle only within this range (min - max)
        min_range = 5.0
        max_range = 70.0
        
        lr = []
        self.mymutex.acquire()
        #Copy range data
        for i in range(210):
            if (self.lidar.LidarRange[i]>min_range and self.lidar.LidarRange[i]<max_range):
                lr.append(self.lidar.LidarRange[i])
            else:
                lr.append(0)

        #Reset lidar data after copying to local variables
        self.lidar.Reset()
        self.mymutex.release()

        #Compute repulsive forces
        #Range only upto index 200 (more than that index is not so reliable)
        forceX = []
        forceY = []
        sumFx = 0
        sumFy = 0
        for i in range(200):
            forceX.append(0)
            forceY.append(0)
            if (lr[i]>0):
                force = -max_range/lr[i]
                forceX[i] = force * self.lidar.LidarCos[i]
                forceY[i] = force * self.lidar.LidarSin[i]
            
            sumFx += forceX[i]
            sumFy += forceY[i]

        force_p_threshold = 75.0
        force_n_threshold =-force_p_threshold

        #Calculate motion in roll (X-direction)
        if (sumFx < force_n_threshold):
            rc2=1400.0
        elif (sumFx > force_p_threshold):
            rc2=1600.0
        else:
            rc2=1500.0 + 200.0/(2.0*force_p_threshold)

        #Calculate motion in pitch (Y-direction)
        if (sumFy < force_n_threshold):
            rc1=1400.0
        elif (sumFy > force_p_threshold):
            rc1=1600.0
        else:
            rc1=1500.0 + 200.0/(2.0*force_p_threshold)

        self.RC_Override2 = int(rc2)
        self.RC_Override1 = int(rc1)

        return


############################## Function Connect device ############################
    def ConnectDevices(self, ConnectLidar, ConnectCamera):
        
        #Connect Pixhawk (Pixhawk)
        try:
            portname = self.FindPixhawk();
            #Connect to pixhawk
            self.vehicle = connect(portname, wait_ready=True, baud=115200)
            print "Connect Pixhawk at port ",
            print portname
        except:
            print "Error: Pixhawk cannot be connected!"
            return 1

        #Connect HardOverride device
        self.hardOverride = HardOverride()
        if (self.hardOverride.Connect()==False):
            return 2

        #Connect Lidar device
        if (ConnectLidar):
            self.lidar = Lidar()
            if (self.lidar.Connect()==False):
                return 3

        #When no error in connection, return 0
        return 0


############################## Function Start ############################
    def Start(self):
        self.mymutex = Lock()
        self.airmutex = Lock()
        
        #Parameters of the aircraft
        self.Aircraft_Armed=0
        self.Aircraft_Height =0
        self.Aircraft_Velocity=[0 for i in range(3)]
        self.AirCraft_Battery=0

        #Start the thread loop
        self.Loop = True

        #Try arming sequence to make aircraft in loiter mode
        armresult = False
        for trial in range(5):
            if self.ArmAircraftLoiter(10, 0.01):  #Make 5 steps, each has 1 second delay
                armresult = True
                break

        if (not armresult):
            print "ERROR: Cannot arm aircraft in loiter mode"
            self.Stop()
            return self.Stop()

        #Start RC override thread
        try:
            thread.start_new_thread(self.RCOverideLoop, ())
        except:
            print "Error: Unable to start RC override loop"
            self.Stop()
            return self.Stop()
        
        #Wait until override loop ready
        time.sleep(1)

        #Start RC height thread
        self.Desired_Height = 0.0
        try:
            thread.start_new_thread(self.RCHeightControlLoop, ())
        except:
            print "Error: Unable to start height control loop"
            self.Stop()
            return self.Stop()

        #Start lidar reading thread
        if (self.lidar != None):
            #Activate Lidar reading and obstacle thread
            try:
                thread.start_new_thread(self.LidarReadingLoop, ())
            except:
                print "Error: Unable to lidar reading loop"
                return self.Stop()
        return True

###################### Function: Stop aircraft and land ###########################
    def Stop(self):
        print "Disarm..."
        try:
            self.ChangeRCVal(10, 0.1, 1500, 1500, 1000, 1000, 2000)
            self.vehicle.armed = False
            time.sleep(1)
        except:
            pass
        return False


############################## Function Take Off ##############################
    def Takeoff(self, Height):
        if self.vehicle==None: return
        print ("Taking off to height=%0.2f")%(Height)
        self.Desired_Height=Height

    
######################  Funciton: Landing Aircraft  ######################
    def Landing(self):
        if self.vehicle==None: return
        print "Landing..."
        self.Desired_Height = 0.0

############################## Function Check aircraft landed ############################
    def IsLanded(self):
        if self.vehicle==None: return True
        try:
            #Lower limit of TR1 range finder is 20 cm above ground
            if (self.Aircraft_Height <= 0.201):
                return True
        except:
            return False
        
        return False
    
################ Function: Close connection ##################
    def Close():
        self.Loop = False
        self.vehicle.close()
        self.hardOverride.Close()
        self.lidar.Close()

################ Function: Cubic polynomial height planning ##################
    def _cubic_coef(self, MaxVel, TimeStep):
        
        h = self.Desired_Height -self.Aircraft_Height
        t = math.fabs(h)/MaxVel
        
        #If time to reach target is too small, stop the aircraft at current height
        if (t<0.001):
            self.errH = 0.0
            self.errV = 0.0
            return
        
        #Calculate cubic coefficient
        tt = t*t    #t^2
        ttt = tt*t  #t^3
        a1 = self.Aircraft_Velocity[2]
        a2 = 3*h/tt  -2*self.Aircraft_Velocity[2]/t
        a3 = 2*h/ttt +3*self.Aircraft_Velocity[2]/tt
        
        #Reuse variable t, tt and ttt
        t  = TimeStep
        tt = TimeStep*TimeStep
        ttt= t*tt
    
        self.errH = a1*t +a2*tt+ a3*ttt
        self.errV = 2*a2*t +3*a3*tt
    
    
############################## Height Control Loop ########################
    def RCHeightControlLoop(self):
        print "Height control loop Starting...\r"
        #Define maximum height error for one time step
        max_err_height = 0.075    #meter
        err_cnt=0
            
        #Declare and initialize array
        err_height_record = []
        for i in range(5):
            err_height_record.append(0)

        batt_cnt=0
        batt_vol=[]
        self.AirCraft_Battery = self.vehicle.battery.voltage
        for i in range(10):
            batt_vol.append(0)
            batt_vol[i] = self.AirCraft_Battery
        
        while (self.Loop):
            #Read aircraft status
            self.Aircraft_Armed          = self.vehicle.armed
            self.Aircraft_Height         = self.vehicle.rangefinder.distance
            self.Aircraft_Velocity       = self.vehicle.velocity
            self.AirCraft_Battery        = self.vehicle.battery.voltage
            
            #Calculate height in next time step with max velocity 1 m/s and 0.1 second time step
            self._cubic_coef(0.5, 0.1)
            
            #Calculate height error
            err_height = self.Desired_Height -self.Aircraft_Height
               
            if (math.fabs(err_height)<0.3):
                max_err_height = 0.025
            else:
                max_err_height = 0.1
                
            #Limit error to maximum height error
            if (math.fabs(err_height)>max_err_height):
                err_height = math.copysign(max_err_height, err_height)
            
            #Accumulate height error
            err_height_record[err_cnt] = err_height
            err_cnt = err_cnt+1
            if (err_cnt>4):
                err_cnt=0

            sum_err = err_height_record[0] +err_height_record[1] +err_height_record[2] +err_height_record[3] +err_height_record[4]

            #Calculate RC value = Middle Position + Kp * err + Ki * sum_err - Kd*velocity
            throttle = 1500 +1500*err_height +20*sum_err -100*self.Aircraft_Velocity[2]
    
            #Limit RC value to be lower than 2000
            if (throttle>2000):
                self.RC_Override3=2000
            else:
                self.RC_Override3=int(throttle)
            
            #Limit RC value to be higher than 1450
            if (throttle<1490):
                self.RC_Override3=1490
            else:
                self.RC_Override3=int(throttle)

            #Update control loop at 10 Hz
            time.sleep(0.1)

            #Check batt status
            batt_vol[batt_cnt] = self.AirCraft_Battery
            batt_stat = batt_vol[0]+batt_vol[1]+batt_vol[2]+batt_vol[3]+batt_vol[4]+batt_vol[5]+batt_vol[6]+batt_vol[7]+batt_vol[8]+batt_vol[9]
            batt_stat = batt_stat/10.0
            if (batt_stat<13.9): print("Warning: Battery Low!")

            batt_cnt = batt_cnt+1
            if (batt_cnt>9): batt_cnt=0

            ##### Print out the battery voltage here ####
            stdout.write("\rBattery Voltage: %d, %0.1f" %(self.RC_Override3, self.AirCraft_Battery))
            #stdout.write("\rErr, Batt, Throttle: %d, %0.2f,%0.2f" %(self.RC_Override3, self.errH, self.errV))
            stdout.flush()
            
            doonce=True
            if (self.Aircraft_Height >0.8): doonce=False
            if (doonce==False):
                #stdout.write("\nErr, Batt, Throttle: %d, %0.2f,%0.2f" %(self.RC_Override3, self.errH, self.errV))
                #stdout.flush()
                doonce=True

            #End while
                    
        print "Height control loop terminated!\r"
        #If exit loop, stay still until all other loops are quit
        self.RC_Override3 = 1500
        return

