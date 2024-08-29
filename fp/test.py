
#Replace this with your HW7 P2 code, edited for this problem!

'''

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np
from std_msgs.msg import Float64
from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain
from demos.TransformHelpers     import *

#
#   Atlas Joint Names
#
jointnames = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
              'l_leg_kny',
              'l_leg_akx', 'l_leg_aky',

              'r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz',
              'r_leg_kny',
              'r_leg_akx', 'r_leg_aky',

              'back_bkx', 'back_bky', 'back_bkz',
              'neck_ry',

              'l_arm_elx', 'l_arm_ely',
              'l_arm_shx', 'l_arm_shz',
              'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',
 
              'r_arm_shz', 'r_arm_shx',
              'r_arm_ely', 'r_arm_elx',
              'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']

#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass
        
        self.chain_rarm = KinematicChain(self, 'pelvis', 'r_hand', ['back_bkz', 'back_bky', 'back_bkx',
                                                                    'r_arm_shz', 'r_arm_shx',
                                                                    'r_arm_ely', 'r_arm_elx',
                                                                    'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2'])
        
        self.chain_lleg = KinematicChain(self, 'l_lleg', 'pelvis', ['l_leg_kny', 'l_leg_hpy', 'l_leg_hpx', 'l_leg_hpz'])


        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Define the various points.        
        # self.q0_rarm = np.radians( np.array([0,0,0, 90., 30., 0., -30., 0.,0.,0.]).reshape((-1,1)) )
        # (self.p0, self.R0, _, _) = self.chain_rarm.fkin(self.q0_rarm)  #np.array( [4., 2., 4.]).reshape((-1,1) )
        #self.R0 = Reye()

        self.q0_lleg =  np.radians( np.array([0.,0.,0.,0.]).reshape((-1,1)))
        (self.p0_pelvis, self.R0_pelvis, _, _) = self.chain_lleg.fkin(self.q0_lleg)  #np.array( [4., 2., 4.]).reshape((-1,1) )
        self.q_lleg = self.q0_lleg
        self.pd_pelvis = self.p0_pelvis
        self.Rd_pelvis = self.R0_pelvis

        self.lam = 20

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update) 
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown.
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt
        #T = 10.0    
        # Compute position/orientation of the pelvis (w.r.t. world).
        ppelvis = pxyz(0.0, 0.0, 0.862)
        Rpelvis = Reye()
        Tpelvis = T_from_Rp(Rpelvis, ppelvis)
        
        # Build up and send the Pelvis w.r.t. World Transform!
        trans = TransformStamped()
        trans.header.stamp    = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id  = 'pelvis'
        trans.transform       = Transform_from_T(Tpelvis)
        self.broadcaster.sendTransform(trans)
        
        q    = np.zeros((len(jointnames), 1)) 
        qdot = np.zeros((len(jointnames), 1))

        pd_pelvis, vd_pelvis = goto(self.t, ppelvis, np.array([-0.395616, -0.1115, 0.637408]), 5.0) 
        Rd_pelvis = Reye()
        wd_pelvis = np.zeros(3).reshape(3,1)

        qlast_lleg = self.q_lleg 
        pdlast = self.pd_pelvis
        Rdlast = self.Rd_pelvis
        (ptip, Rtip, Jv, Jw) = self.chain_rarm.fkin(qlast_lleg) 
        
        J = np.vstack((Jv, Jw)) 

        eP = ep(pdlast, ptip) 
        er = eR(Rdlast, Rtip) 
        error = np.vstack((eP,er)) 
        x_dot = np.vstack((vd_pelvis, wd_pelvis)) 

       
        # Cutting trajectory 
        # pd = pxyz(0.1 * np.sin(3*self.t) + 0.6, -0.33, 0.268) 
        # vd = pxyz(0.3 * np.cos(3*self.t), 0, 0)
        # Rd = Rotz(pi/2)
        # wd = np.array([0,0,0]).reshape(3,1)

        

        # qlast_rarm = self.q_rarm 
        # pdlast = self.pd
        # Rdlast = self.Rd
        # (ptip, Rtip, Jv, Jw) = self.chain_rarm.fkin(qlast_rarm) 
        
    #     J = np.vstack((Jv, Jw)) 

    #     eP = ep(pdlast, ptip) 
    #     er = eR(Rdlast, Rtip) 
    #     error = np.vstack((eP,er)) 
    #     x_dot = np.vstack((vd, wd)) 

    #     # 10 total joints
    #     W = np.diag([1/100, 1/100, 1/100, 1/2, 1, 1, 1, 1, 1, 1])
    #     W_inv = np.linalg.inv(W)

    #    # W_inv = np.diag([0, 0, 0, 1, 1, 1, 1, 1, 1, 1])
        
    #     J_W = J @ W_inv
    #     J_W_transpose = np.transpose(J_W)
    #     J_W_inv = J_W_transpose @ np.linalg.inv(J_W @ J_W_transpose)
        
        
        # lam_tert = 100
        
        # #Twist bkz such that right arm goes away from cut point
        # qgoal_tert = 0.6 *np.sin(1*self.t)
        # #print('qgoal_sec', qgoal_sec, '\n')
        # qdot_tert_back = lam_tert * (qgoal_tert - self.q_rarm[2])
        # qdot_tert = np.array([ 0,0, qdot_tert_back[0],0,0,0,0,0,0,0]).reshape(10,1)
        # #(np.append(qdot_tert_back,[np.zeros((9,1))])).reshape(10,1)

        #  ['back_bkz', 'back_bky', 'back_bkx',
        # 'r_arm_shz', 'r_arm_shx',
        # 'r_arm_ely', 'r_arm_elx',
        # 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']

        # Added 7th task coordinate
        # J_7 = np.array([-1.17,0,0,1,0,0,0,0,0,0])
        # J = np.vstack((J,J_7))

        # xd_7 = np.array([0])
        # xtip_7 = np.array([1.17*qlast_rarm[0] + qlast_rarm[3]])

        # v7 = np.array([0])
        # x_dot = np.vstack((x_dot, v7))    

        # ep_7 = ep(xd_7, xtip_7)
        # error = np.vstack((error, ep_7))
        
        #print(qlast_rarm[3])
        # 2ndary task method of keeping back still
        # lam_sec = 100
        # qgoal_sec = np.zeros((3,1))
        # qdot_sec_back = lam_sec * (qgoal_sec - self.q_rarm[:3])
        # qdot_sec = (np.append(qdot_sec_back,[np.zeros((7,1))])).reshape(10,1)

        Jinv = np.linalg.pinv(J)
        #qdot_rarm = J_W_inv @ (x_dot + self.lam * error)
        #qdot_rarm = Jinv @ (x_dot + self.lam * error)

        qdot_lleg = Jinv @ (x_dot + self.lam * error)  # + (np.identity(10) - Jinv @ J) @ qdot_sec  + (np.identity(10) - Jinv @ J) @ qdot_tert
        q_lleg = qlast_lleg + qdot_lleg * self.dt 
        self.q_lleg = q_lleg 
        self.pd_pelvis = pd_pelvis 
        self.Rd_pelvis = Rd_pelvis
        
        #sec
        #x_sec = 
        #qdot_sec = lam_sec * 

        '''
        gamma = 0.05
        Jinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6)) 
        
        qdot_rarm = Jinv @ (x_dot + self.lam * error) 
        q_rarm = qlast_rarm + qdot_rarm * self.dt 
        self.q_rarm = q_rarm 
        self.pd = pd 
        self.Rd = Rd
        '''
        
        # Right Arm 
        #print(q_rarm)
        q[0:3] = q_lleg
        qdot[0:3] =  qdot_lleg

        # q[12:15] = q_rarm[:3] 
        # q[23:] =  q_rarm[3:]
        # qdot[12:15] = qdot_rarm[:3] 
        # qdot[23:] = qdot_rarm[3:]

        #print(q)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = jointnames                # List of names
        cmdmsg.position     = q.flatten().tolist()      # List of positions
        cmdmsg.velocity     = qdot.flatten().tolist()   # List of velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    node = DemoNode('cut', 100)
    #node = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
