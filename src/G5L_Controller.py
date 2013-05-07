#! /usr/bin/env python
import roslib; roslib.load_manifest('5link')
import math, rospy, os, rosparam
from RobotController.msg import * 
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from numpy import zeros, array, linspace, arange
import numpy as np
from JointController import JointCommands_msg_handler
from robot_state import robot_state
from math import ceil
import yaml
from copy import copy
from std_srvs.srv import Empty

class Fivel_Controller(object):
    """Fivel_Controller"""
    def __init__(self, arg):
        super(Fivel_Controller, self).__init__()

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################
        if len(arg)<10:
            self.SwingEff0 = -1.2             # Default swing effort
            self.SupAnkSetPoint = -0.04       # Setpoint for support ankle
            self.SupAnkGain_p = 30            # Support ankle gain p
            self.SupAnkGain_d = 10            # Support ankle gain d
            self.SwingEffDur = 0.50           # Duration of swing effort
            self.FootExtEff = 1.0             # Foot extension effort
            self.FootExtDur = 0.22            # Foot extension duration
            self.ToeOffEff = 17.65            # Toe-off effort
            self.ToeOffDur = 0.24             # Toe-off duration
            self.DesTorsoAng = -0.055         # Desired torso angle
        else:
            self.SwingEff0 = arg[0]           # Default swing effort
            self.SupAnkSetPoint = arg[1]      # Setpoint for support ankle
            self.SupAnkGain_p = arg[2]        # Support ankle gain p
            self.SupAnkGain_d = arg[3]        # Support ankle gain d
            self.SwingEffDur = arg[4]         # Duration of swing effort
            self.FootExtEff = arg[5]          # Foot extension effort
            self.FootExtDur = arg[6]          # Foot extension duration
            self.ToeOffEff = arg[7]           # Toe-off effort
            self.ToeOffDur = arg[8]           # Toe-off duration
            self.DesTorsoAng = arg[9]         # Desired torso angle
        
        self.StepsTaken = 0
        self.ApertureMean = 0
        self.ApertureStd = 0
        self.GlobalPos = 0

        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)

        ##################################################################
        ###################### RAISE FOOT SEQUENCE #######################
        ##################################################################

        self.pos1=zeros(4)

        # "left_hip","left_ankle","right_hip","right_ankle"
        #      0           1           2            3     

        # Sequence 1 for raising the left foot (from home position)
        self.seq1_1=copy(self.pos1)
        self.seq1_1[3] = -0.05
        self.seq1_1[0] = 0.25
        self.seq1_1[1] = -0.25

        self.seq1_2=copy(self.seq1_1)
        self.seq1_2[1] = -1.49

        # Sequence 2 for raising the right foot (from home position)
        self.seq2_1=copy(self.pos1)
        self.seq2_1[0] = 0.04
        self.seq2_1[1] = -0.04
        self.seq2_1[2] = 0.25
        self.seq2_1[3] = -0.25

        self.seq2_2=copy(self.seq2_1)
        self.seq2_2[3] = -1.49

        ##################################################################
        ########################## INITIALIZE ############################
        ##################################################################

        self.RobotName = "ghost_fivel"
        self.JointNames = ["left_hip","left_ankle","right_hip","right_ankle"]

        # Initialize joint commands handler
        self.JC = JointCommands_msg_handler(self.RobotName,self.JointNames)

        # Initialize robot state listener
        self.RS = robot_state(self.JointNames)
        self.MsgSub = rospy.Subscriber('/'+self.RobotName+'/robot_state',RobotState,self.RS_cb)
        self.OdomSub = rospy.Subscriber('/ground_truth_odom',Odometry,self.Odom_cb)


    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    # def RaiseLeg(self,which):
    #     if which == "inner":
    #         self.JC.send_pos_traj(self.pos1,self.seq1_1,1,0.01)
    #         rospy.sleep(0.2)
    #         self.JC.send_pos_traj(self.seq1_1,self.seq1_2,1,0.01)
    #         self.JC.set_eff('inner_ankle',-5)
    #         self.JC.send_pos_traj(self.seq1_2,self.seq1_3,1,0.01)
    #         rospy.sleep(0.2)
    #     if which == "outer":
    #         self.JC.send_pos_traj(self.pos1,self.seq2_1,2,0.01)
    #         rospy.sleep(0.2)
    #         self.JC.send_pos_traj(self.seq2_1,self.seq2_2,2,0.01)
    #         self.JC.set_eff('left_ankle',-5)
    #         self.JC.set_eff('right_ankle',-5)
    #         self.JC.send_command()
    #         #self.JC.send_pos_traj(self.seq2_2,self.seq2_3,1,0.01)
    #         rospy.sleep(0.2)

    def StandOn(self,leg,setpoint,time,dt):
        StartTime = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - StartTime < time:
            # Apply closed loop torque for upright torso
            hip_eff = self.BalanceCmd(leg,setpoint)

            self.JC.set_eff(leg+'_hip',hip_eff)
            self.JC.send_command()
            rospy.sleep(dt)

    def BalanceCmd(self,which,setpoint):
        ank_pos = self.RS.GetJointPos(which+"_ankle")
        hip_pos = self.RS.GetJointPos(which+"_hip")
        ank_vel = self.RS.GetJointVel(which+"_ankle")
        hip_vel = self.RS.GetJointVel(which+"_hip")

        torso_angle = ank_pos + hip_pos
        torso_angvel = ank_vel + hip_vel

        # Apply closed loop torque for upright torso
        hip_eff = 200*(setpoint-torso_angle) + 50*(0-torso_angvel)
        return hip_eff


    def TakeStep(self,which):
        #Aperture = abs(self.RS.GetJointPos('left_hip')-self.RS.GetJointPos('right_hip'))
        SwingEff = self.SwingEff0 #+ self.kAp * (self.DesAp - Aperture)
        DesTorsoAng = self.DesTorsoAng
        # if which == "left":
        #     DesTorsoAng = self.DesTorsoAng - 0.1*self.RS.GetJointPos('right_hip')
        # if which == "right":
        #     DesTorsoAng = self.DesTorsoAng - 0.1*self.RS.GetJointPos('left_hip')

        if which == "left":
            # Swing leg 0.4 sec and incline stance leg
            self.JC.set_eff("left_hip",SwingEff)
            self.JC.send_command()
            self.StandOn("right",DesTorsoAng,self.SwingEffDur,0.005)
            # Straighten ankle and free swing
            self.JC.set_eff("left_ankle",self.FootExtEff)
            self.JC.set_eff("left_hip",0)
            self.JC.send_command()
            self.StandOn("right",DesTorsoAng,self.FootExtDur,0.005)
            # Toe off
            self.JC.set_eff("left_ankle",0)
            self.JC.set_pos("left_ankle",self.SupAnkSetPoint)
            self.JC.set_gains("left_ankle",self.SupAnkGain_p,0,self.SupAnkGain_d)
            self.JC.set_eff("right_ankle",self.ToeOffEff)
            self.JC.send_command()
            self.StandOn("left",DesTorsoAng,self.ToeOffDur,0.005)
            # Bend foot for clearance
            self.JC.set_eff("right_ankle",-2)
            self.JC.set_eff("right_hip",0)
            self.JC.send_command()

        if which == "right":
            # Swing leg 0.4 sec and incline stance leg
            self.JC.set_eff("right_hip",SwingEff)
            self.JC.send_command()
            self.StandOn("left",DesTorsoAng,self.SwingEffDur,0.005)
            # Straighten ankle and free swing
            self.JC.set_eff("right_ankle",self.FootExtEff)
            self.JC.set_eff("right_hip",0)
            self.JC.send_command()
            self.StandOn("left",DesTorsoAng,self.FootExtDur,0.005)
            # Toe off
            self.JC.set_eff("right_ankle",0)
            self.JC.set_pos("right_ankle",self.SupAnkSetPoint)
            self.JC.set_gains("right_ankle",self.SupAnkGain_p,0,self.SupAnkGain_d)
            self.JC.set_eff("left_ankle",self.ToeOffEff)
            self.JC.send_command()
            self.StandOn("right",DesTorsoAng,self.ToeOffDur,0.005)
            # Bend foot for clearance
            self.JC.set_eff("left_ankle",-2)
            self.JC.set_eff("left_hip",0)
            self.JC.send_command()

    # def TakeStep(self,which):
    #     TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

    #     if TimeElapsed<60:
    #         Aperture = abs(self.RS.GetJointPos('left_hip'))
    #         self.SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
    #         #self.ToeOffEff = self.ToeOffEff0 - 2*(self.DesAp - Aperture)
            
    #         if which == 'outer':
				# Aperture = self.RS.GetJointPos('left_hip')
    #         else:
				# Aperture = -self.RS.GetJointPos('left_hip')
    #         self.ToeOffEff = self.ToeOffEff0 - 1*Aperture
    #         #self.SupAnkSetPoint = -0.02 + 0.03 * Aperture
			
    #         print Aperture
    #     else:
    #         Aperture = abs(self.RS.GetJointPos('left_hip'))
    #         #self.ToeOffEff = self.ToeOffEff0 - 2*(self.DesAp - Aperture)
    #         #print self.ToeOffEff
    #         #print self.SwingEff
    #     if which == "outer":
    #         # Toe off
    #         self.JC.set_gains('inner_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
    #         self.JC.set_pos('inner_ankle',self.SupAnkSetPoint)
    #         self.JC.set_eff('left_ankle',self.ToeOffEff)
    #         self.JC.set_eff('right_ankle',self.ToeOffEff)
    #         self.JC.set_eff('left_hip',0)
    #         self.JC.set_eff('right_hip',0)
    #         self.JC.send_command()
    #         rospy.sleep(self.ToeOffDur)
    #         # Raise feet and swing
    #         self.JC.set_eff('left_ankle',-3)
    #         self.JC.set_eff('right_ankle',-3)
    #         self.JC.set_eff('left_hip',-self.SwingEff)
    #         self.JC.set_eff('right_hip',-self.SwingEff)
    #         self.JC.send_command()
    #         rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
    #         # Lower feet
    #         self.JC.set_eff('left_ankle',self.FootExtEff)
    #         self.JC.set_eff('right_ankle',self.FootExtEff)
    #         self.JC.send_command()
    #         rospy.sleep(self.FootExtDur)
    #         # End swing
    #         self.JC.set_eff('left_hip',0)
    #         self.JC.set_eff('right_hip',0)
    #         self.JC.set_eff('left_ankle',0)
    #         self.JC.set_eff('right_ankle',0)
    #         self.JC.send_command()
    #         rospy.sleep(self.FreeSwingDur)

    #     if which == "inner":
    #         # Toe off
    #         self.JC.set_gains('left_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
    #         self.JC.set_gains('right_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
    #         self.JC.set_pos('left_ankle',self.SupAnkSetPoint)
    #         self.JC.set_pos('right_ankle',self.SupAnkSetPoint)
    #         self.JC.set_eff('inner_ankle',self.ToeOffEff)
    #         self.JC.set_eff('left_hip',0)
    #         self.JC.set_eff('right_hip',0)
    #         self.JC.send_command()
    #         rospy.sleep(self.ToeOffDur)
    #         # Raise feet and swing
    #         self.JC.set_eff('inner_ankle',-3)
    #         self.JC.set_eff('left_hip',self.SwingEff)
    #         self.JC.set_eff('right_hip',self.SwingEff)
    #         self.JC.send_command()
    #         rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
    #         # Lower feet
    #         self.JC.set_eff('inner_ankle',self.FootExtEff)
    #         self.JC.send_command()
    #         rospy.sleep(self.FootExtDur)
    #         # End swing
    #         self.JC.set_eff('left_hip',0)
    #         self.JC.set_eff('right_hip',0)
    #         self.JC.set_eff('inner_ankle',0)
    #         self.JC.send_command()
    #         rospy.sleep(self.FreeSwingDur)

    def ResetPose(self):
        self.JC.set_pos('left_hip',0)
        self.JC.set_pos('left_ankle',0)
        self.JC.set_pos('right_hip',0)
        self.JC.set_pos('right_ankle',0)
        self.JC.send_command()

    def RS_cb(self,msg):
        self.RS.UpdateState(msg)

    def Odom_cb(self,msg):
        self.GlobalPos = msg.pose.pose.position

    def reset(self):
        self.reset_srv()
        rospy.sleep(1.5)

        while self.GlobalPos.z<0.97 or self.GlobalPos.z>1.03 or abs(self.GlobalPos.x)>0.5:
            self.reset_srv()
            rospy.sleep(1)

    def Run(self,TimeOut = 0):
        rospy.sleep(0.1)

        # Start with left leg raised
        # self.JC.send_pos_traj(self.RS.GetJointPos(),self.seq1_2,0.5,0.01) 
        # rospy.sleep(0.5)
        # self.reset()
        # self.StandOn("right",10,0.005)

        # Start with right leg raised
        self.JC.set_gains('right_hip',2000,0,150)
        self.JC.set_gains('left_hip',2000,0,150)
        self.JC.set_gains('left_ankle',250,0,20)
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.seq2_2,0.5,0.01) 
        rospy.sleep(0.5)
        self.JC.reset_gains()
        self.reset()
        rospy.sleep(2)

        self.StartTime = rospy.Time.now().to_sec()

        self.JC.set_pos("left_ankle",self.SupAnkSetPoint)
        self.JC.set_gains("left_ankle",self.SupAnkGain_p,0,self.SupAnkGain_d)

        self.Leg = "right"
        self.Go=1
        while self.Go == 1:
            TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

            if self.GlobalPos.z<0.6 or self.GlobalPos.z>1.4 or (TimeElapsed>TimeOut and TimeOut>0):
                # if the robot fell, stop running and return fitness
                r,p,y = self.RS._orientation.GetRPY()
                self.GlobalPos.x += 1*math.sin(self.RS.GetJointPos(self.Leg+'_hip')-p)
                return self.GlobalPos, self.ApertureStd

            self.TakeStep(self.Leg)

            self.StepsTaken += 1
            ThisAperture = abs(self.RS.GetJointPos('left_hip')-self.RS.GetJointPos('right_hip'))
            print ThisAperture
            self.ApertureMean += (ThisAperture-self.ApertureMean) / self.StepsTaken
            self.ApertureStd += ((ThisAperture-self.ApertureMean)**2 - self.ApertureStd) / self.StepsTaken

            if self.Leg == "right":
                self.Leg = "left"
            else:
                self.Leg = "right"


##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################

if __name__=='__main__':
    Fivel_C = Fivel_Controller([])
    A,B = Fivel_C.Run(60)
    print A,B
