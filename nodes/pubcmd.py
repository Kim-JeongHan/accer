#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
# from sol_msg.msg import track
from geometry_msgs.msg import Twist, Vector3
# from track.srv import PlaySong, PlaySongResponse  

class CmdVel():
    x = 0.0
    y = 0.0
    w = 0.0

class AccerParam():
    update_ratio = 0.01  ##sec
    max_acceration = 0.8 ## m/s^2
    # min_acceration = 
    purpose_time = 0.5 ##sec

class VisionControlNode:
    def __init__(self):
        rospy.Subscriber('stop', Bool, self.stop_callback)
        # rospy.Subscriber('tracker', track, self.tracker_callback)
        self.cmd_publisher=rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
        
        self.cmd_vel = CmdVel()
        self.purpose = CmdVel()
        self.Priority = 0
        self.accerParam = AccerParam()

        rospy.Timer(rospy.Duration(self.accerParam.update_ratio), self.Acceration)
        rospy.Timer(rospy.Duration(0.1), self.cmd) ## 100hz pub


    def main(self):
        rospy.spin()
    
    def __del__(self):
        pass

    def Acceration(self, event=None):
        max_dv = self.accerParam.max_acceration*self.accerParam.update_ratio  
        #max_vel = self.accerParam.max_acceration*self.accerParam.purpose_time

        

        xdiff = self.purpose.x - self.cmd_vel.x
        wdiff = self.purpose.w - self.cmd_vel.w
        dv_x = xdiff*self.accerParam.update_ratio/self.accerParam.purpose_time
        dv_w = wdiff*self.accerParam.update_ratio/self.accerParam.purpose_time
        if xdiff > 0 :
            self.cmd_vel.x = self.cmd_vel.x + dv_x #  0.05m/s
        elif xdiff == 0:
            self.cmd_vel.x = self.purpose.x
        else: 
            self.cmd_vel.x = self.cmd_vel.x - dv_x

        if wdiff > 0 :
            self.cmd_vel.w = self.cmd_vel.w+ dv_w #  0.05m/s
        elif wdiff == 0:
            self.cmd_vel.w=self.purpose.w
        else:
            self.cmd_vel.w = self.cmd_vel.w- dv_w

    def stop_callback(self, data):
        if (data.data == True):
            self.Priority = 1
            # self.purpose = 0

        else:
            self.Priority = 2
            # stop = 0

    # def tracker_callback(self, data):
    #     if(self.Priority == 2):
    #         if data.position[0] <260:
    #             rospy.loginfo('turn right')
    #             self.purpose.x = 0.1
    #             self.purpose.w = 0.2

    #         elif data.position[0] <410:
    #             rospy.loginfo('go')
    #             self.purpose.x = 0.2
    #             self.purpose.w = 0.0
    #         else:
    #             rospy.loginfo('turn left')
    #             self.purpose.x = 0.1
    #             self.purpose.w = -0.2



    def cmd(self,event=None):
        self.purpose.x = 2
        self.purpose.w = 2
        cmdvel=Twist(Vector3(self.cmd_vel.x, self.cmd_vel.y, 0), Vector3(0,0,self.cmd_vel.w))
        self.cmd_publisher.publish(cmdvel)
    
if __name__ == '__main__':
    rospy.init_node('VisionControlNode', anonymous=True)
    node = VisionControlNode()
    node.main()