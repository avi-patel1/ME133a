
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

              'back_bkz', 'back_bky', 'back_bkx',
              'neck_ry',

              'l_arm_shz', 'l_arm_shx',
              'l_arm_ely', 'l_arm_elx',
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
        
        self.chain_larm = KinematicChain(self, 'pelvis', 'l_hand', ['back_bkz', 'back_bky', 'back_bkx',
                                                                    'l_arm_shz', 'l_arm_shx',
                                                                    'l_arm_ely', 'l_arm_elx',
                                                                    'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2'])

        
        # self.chain_lleg = KinematicChain(self, 'l_lleg', 'pelvis', ['l_leg_kny', 'l_leg_hpy', 'l_leg_hpx', 'l_leg_hpz'])
        # self.chain_rleg = KinematicChain(self, 'pelvis', 'r_lleg', ['r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz', 'r_leg_kny'])


        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Define the various points.
        self.q0_back = np.array([0.0, 0.0, 0.0]).reshape(3,1)

        self.q0_rarm = np.radians( np.array([90., 30., 0., -30., 0.,0.,0.]).reshape((-1,1)) )

        self.q0_larm = np.radians( np.array([-90., -30., 0., 30., 0.,0.,0.]).reshape((-1,1)))
        

        self.q_rarm = self.q0_rarm # Right arm
        self.q_larm = self.q0_larm # Left arm

        self.q = np.vstack( (self.q0_back, np.vstack((self.q_rarm, self.q_larm))))


        (self.p0_rarm, self.R0_rarm, _, _) = self.chain_rarm.fkin(self.q[0:10])  #np.array( [4., 2., 4.]).reshape((-1,1) )
        (self.p0_larm, self.R0_larm, _, _) = self.chain_larm.fkin(np.vstack((self.q[0:3], self.q[10:])))  #np.array( [4., 2., 4.]).reshape((-1,1) )


        self.pd_r = self.p0_rarm
        self.Rd_r = self.R0_rarm

        self.pd_l = self.p0_larm
        self.Rd_l = self.R0_larm


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
        

        # Define q and qdot
        q    = np.zeros((len(jointnames), 1)) 
        qdot = np.zeros((len(jointnames), 1))


        # Right Arm Cutting Trajectory 
        # T_from_URDF_origin(pzero()) @
        pd_r =  pxyz(0.1 * np.sin(3*self.t) + 0.6, -0.33, 0.268) # subtract from pelvis
        vd_r =  pxyz(0.3 * np.cos(3*self.t), 0, 0)
        Rd_r = Rotz(pi/2)
        wd_r = np.array([0,0,0]).reshape(3,1)


        qlast_r = self.q[:10] 
        pdlast_r = self.pd_r
        Rdlast_r = self.Rd_r
        (ptip_r, Rtip_r, Jv_r, Jw_r) = self.chain_rarm.fkin(qlast_r)  # Right Arm 

        eP_r = ep(pdlast_r, ptip_r) 
        er_r = eR(Rdlast_r, Rtip_r) 
        error_p = np.vstack((eP_r,er_r)) 
        xdot_p = np.vstack((vd_r, wd_r)) 

        J_p =  np.vstack((Jv_r, Jw_r)) 
        J_p = np.hstack((J_p, np.zeros((6,7))))
        print("Shape of J_p", J_p.shape)
        
        Jinv_p = np.linalg.pinv(J_p)

        # Trajectory of left arm (spline motion)
        sp    =   sin(0.2 * self.t) 
        spdot = 0.2 * cos(0.2 * self.t)

       # ''' Simple Trajectory 
        pf, Rf, _, _ = self.chain_larm.fkin(np.array([0, 0, 0, 0., -30., 0., 30., 0., 0., 0.]).reshape((-1,1))) 
        pd_l = 0.5 * (self.p0_larm + pf)  + 0.5 * (pf - self.p0_larm) * sp
        vd_l = 0.5 * (pf - self.p0_larm) * spdot
       # '''

        
        # ''' Singularity Case
        # pf = pxyz(-0.3777-0.5, 0.6200, 1.29068) - ppelvis
        # pd_l = 0.5 * (self.p0_larm + pf)  + 0.5 * (pf - self.p0_larm) * sp
        # vd_l =  pxyz(0.,0.,0.) #0.5 * (pf - self.p0_larm) * spdot
        # '''

        #pf, Rf, _, _ =  self.chain_larm.fkin(np.array([0, 0, 0, 45., 0., 0., 0., 0., 0., 0.]).reshape((-1,1))) 
        # pf = pxyz(-0.3777-0.5, 0.6200, 1.29068) - ppelvis
        



        # pd_l_f = np.array([0.115548, 0.71051, 1.21365]).reshape(-1,1) # Final desired position for left arm 
        # pd_l, vd_l = goto(self.t, self.p0_larm, pd_l_f, 5.0)
        # pd_l = pxyz(0.128, 0.2 * np.sin(1*self.t) + 0.6, 0.428) # subtract from pelvis
        # vd_l = pxyz(0, 0.2 * np.cos(1*self.t), 0)
        Rd_l = Rotz(np.pi/2)
        wd_l = np.array([0,0,0]).reshape(3,1)

        
        qlast_l = np.vstack((self.q[0:3], self.q[10:]))
        print(qlast_l)
        pdlast_l = self.pd_l
        Rdlast_l = self.Rd_l
        (ptip_l, Rtip_l, Jv_l, Jw_l) = self.chain_larm.fkin(qlast_l) # Left Arm fkin
        eP_l = ep(pdlast_l, ptip_l) 
        er_l = eR(Rdlast_l, Rtip_l) 
        error_s = np.vstack((eP_l, er_l)) 
        xdot_s = np.vstack((vd_l, wd_l)) 
        
        J_s = np.vstack((Jv_l, Jw_l)) 
        J_s = np.hstack((J_s[:,:3], np.zeros((6,7)), J_s[:,3:]))
        print("shape of J_s", J_s.shape)

        W_s_inv = np.diag([.1,.1,.1,1,1,1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]) # Scaling Joint velocities

        J_sW = J_s @ W_s_inv
        J_sW_transpose = np.transpose(J_sW)

        gamma = 0.5
        J_sW_inv = J_sW_transpose @ np.linalg.pinv(J_sW @ J_sW_transpose + gamma**2 * np.eye(6))

        #J_sW_inv = (W_inv @ J_sW_transpose) @ np.linalg.inv(J_sW @ J_sW_transpose)
        #Jinv_s = np.linalg.pinv(J_s)

        #print(x_dot.shape, error.shape)
        #qdot_rarm = J_W_inv @ (x_dot + self.lam * error)
        #qdot_rarm = Jinv @ (x_dot + self.lam * error)

        W_inv = np.diag([.1,.1,.1,1,1,1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]) # Scaling Joint velocities
        J_pW = J_p @ W_inv
        J_pW_transpose = np.transpose(J_pW)
        J_pW_inv = J_pW_transpose @ np.linalg.inv(J_pW @ J_pW_transpose)

        lam_s = 20 
        qdot_s = lam_s * (W_s_inv @ J_sW_inv) @ error_s

        qdot_p = (W_inv @ J_pW_inv ) @ (xdot_p + self.lam * error_p) + (np.identity(17) - Jinv_p @ J_p) @ qdot_s 
        print("qdot_p", Jinv_p @ (xdot_p + self.lam * error_p) )
        print("qdot_s", (np.identity(17) - Jinv_p @ J_p) @ qdot_s )
        qlast =  np.vstack((qlast_r, qlast_l[3:]))
        self.q = qlast + qdot_p * self.dt  
        self.pd_r = pd_r
        self.Rd_r = Rd_r
        self.pd_l = pd_l
        self.Rd_l = Rd_l

        # self.q_larm = qlast_l + qdot_s * self.dt

        ### Publish Joints  

        # Right Arm 
        q[23:] =  self.q[3:10]
        qdot[23:] = qdot_p[3:10]

        # Left Arm
        q[16:23] = self.q[10:]
        qdot[16:23] = qdot_p[10:]

        # Back Joints 
        q[12:15] = self.q[0:3] 
        qdot[12:15] = qdot_p[0:3]

        # qlast_arm  =np.vstack((qlast_rarm, qlast_larm)) 
        # J_top = np.hstack((J_r, np.zeros((6,7))))
        # J_bot = np.hstack((np.zeros((6,7)), J_l))
        # J = np.vstack((J_top, J_bot))

    
        # error = np.vstack((error_r, error_l))
        # x_dot = np.vstack((x_dot_r, x_dot_l))

       

        # 10 total joints
        # W = np.diag([1/100, 1/100, 1/100, 1/2, 1, 1, 1, 1, 1, 1])
        # W_inv = np.linalg.inv(W)

       # W_inv = np.diag([0, 0, 0, 1, 1, 1, 1, 1, 1, 1])
        
        # J_W = J @ W_inv
        # J_W_transpose = np.transpose(J_W)
        # J_W_inv = J_W_transpose @ np.linalg.inv(J_W @ J_W_transpose)
        
        # lam_tert = 100
        
        # #Twist bkz such that right arm goes away from cut point
        # qgoal_tert = 0.663 * np.sin(1*self.t)
        # #print('qgoal_sec', qgoal_sec, '\n')
        # qdot_tert_back = lam_tert * (qgoal_tert - self.q_rarm[2])
        # qdot_tert = np.array([ 0,0, qdot_tert_back[0] , 0,0,0,0,0,0,0]).reshape(10,1)

        #(np.append(qdot_tert_back,[np.zeros((9,1))])).reshape(10,1)

        #  ['back_bkz', 'back_bky', 'back_bkx',
        # 'r_arm_shz', 'r_arm_shx',
        # 'r_arm_ely', 'r_arm_elx',
        # 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']

        # Added 7th task coordinate
        # J_7 = np.array([-0.05,0,1,0,0,0,0,0,0,0])
        # J = np.vstack((J,J_7))

        # xd_7 = np.array([0])
        # xtip_7 = np.array([0.05*qlast_rarm[0] + qlast_rarm[2]])

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

       
        
        # 
        
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
        
        # # Right Arm 
        # q[23:] =  q_rarm
        # qdot[23:] = qdot_rarm

        # # Left Arm
        # q[16:23] = q_larm[3:]
        # qdot[16:23] = qdot_larm[3:]


        # q[12:15] = q_larm[:3] 
        # # q[23:] =  q_arm[3:]
        # qdot[12:15] = qdot_larm[:3] 
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
