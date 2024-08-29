
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

# class Trajectory():
#     # Initialization.
#     def __init__(self, node):
        
#                 # Initialize the transform broadcaster
#         #self.broadcaster = TransformBroadcaster(self)

#         # Add a publisher to send the joint commands.
#         #self.pub = self.create_publisher(JointState, '/joint_states', 10)

#         # Wait for a connection to happen.  This isn't necessary, but
#         # means we don't start until the rest of the system is ready.
#         # self.get_logger().info("Waiting for a /joint_states subscriber...")
#         # while(not self.count_subscribers('/joint_states')):
#         #     pass
        
#         self.chain_rarm = KinematicChain(node, 'r_scap', 'r_hand', ['r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2'])

#         # Set up the timing so (t=0) will occur in the first update
#         # cycle (dt) from now.
#         # self.dt    = 1.0 / float(rate)
#         # self.t     = -self.dt
#         # self.start = self.get_clock().now() + Duration(seconds=self.dt)

#         # Define the various points.        
#         self.q0_rarm = np.radians( np.array([ 30., 50., 0., 0., 0]).reshape((-1,1)) )
#         (self.p0, self.R0, _, _) = self.chain_rarm.fkin(self.q0_rarm)  #np.array( [4., 2., 4.]).reshape((-1,1) )
#         #self.R0 = Reye()

#         self.pboard = np.array([5.,5.,0.]).reshape(-1,1)

#         self.q_rarm = self.q0_rarm
#         self.pd = self.p0 
#         self.Rd = self.R0

#         self.lam = 20

#         # Create a timer to keep calling update().
#         # self.create_timer(self.dt, self.update) 
#         # self.get_logger().info("Running with dt of %f seconds (%fHz)" %
#         #                        (self.dt, rate))


#     # Declare the joint names.
#     def jointnames(self):
#         # Return a list of joint names FOR THE EXPECTED URDF!
#         return jointnames

#     # Evaluate at the given time.  This was last called (dt) ago.
#     def evaluate(self, t, dt):

#         T = 10.0    
#         # Compute position/orientation of the pelvis (w.r.t. world).
#         # ppelvis = pxyz(0.0, 0.5, 1.5 + 0.5 * sin(self.t/2))
#         # Rpelvis = Rotz(sin(self.t))
#         # Tpelvis = T_from_Rp(Rpelvis, ppelvis)
        
#         # Build up and send the Pelvis w.r.t. World Transform!
#         # trans = TransformStamped()
#         # trans.header.stamp    = self.now().to_msg()
#         # trans.header.frame_id = 'world'
#         # trans.child_frame_id  = 'pelvis'
#         # #trans.transform       = Transform_from_T(Tpelvis)
#         # self.broadcaster.sendTransform(trans)
        

#        #t = self.t % T
#         if t < 5: # Approach cutting board 

#             pd, vd = goto(t, 5.0, self.p0, self.pboard)
            
#             #pd = self.p0 + (self.pboard - self.p0) * s0
#             #vd = (self.pboard - self.p0) * s0dot
#             Rd = Reye()
#             wd = np.zeros(3).reshape(-1,1)


#             # Compute the joints.           
#             q    = np.zeros((len(jointnames), 1)) 
#             qdot = np.zeros((len(jointnames), 1))

#             qlast_rarm = self.q_rarm 
#             pdlast = self.pd
#             Rdlast = self.Rd
#             (ptip, Rtip, Jv, Jw) = self.chain_rarm.fkin(qlast_rarm) 
            
#             J = np.vstack((Jv, Jw)) 

#             eP = ep(pdlast, ptip) 
#             er = eR(Rdlast, Rtip) 
#             error = np.vstack((eP,er)) 
#             x_dot = np.vstack((vd, wd)) 

#             gamma = 0.5
#             Jinv = np.linalg.pinv(J)# J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6)) 
            
#             qdot_rarm = Jinv @ (x_dot + self.lam * error) 
#             q_rarm = qlast_rarm + qdot_rarm * dt 
#             self.q_rarm = q_rarm 
#             self.pd = pd 
#             self.Rd = Rd
            
#             # Right Arm 
#             q[25:] =  q_rarm
#             qdot[25:] = qdot_rarm 

#         else:

#             return None

#         # Return the position and velocity as python lists.
#         return (q.flatten().tolist(), qdot.flatten().tolist())

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
        
        self.chain_rarm = KinematicChain(self, 'utorso', 'r_hand', ['r_arm_shz', 'r_arm_shx',
                                                                            'r_arm_ely', 'r_arm_elx',
                                                                            'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2'])

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Define the various points.        
        self.q0_rarm = np.radians( np.array([ 90., 30., 0., -30., 0.,0.,0.]).reshape((-1,1)) )
        (self.p0, self.R0, _, _) = self.chain_rarm.fkin(self.q0_rarm)  #np.array( [4., 2., 4.]).reshape((-1,1) )
        #self.R0 = Reye()

        self.pboard = np.array([.8,-.337,1.13]).reshape(-1,1)

        self.q_rarm = self.q0_rarm
        self.pd = self.p0 
        self.Rd = self.R0

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
        T = 10.0    
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
        

        # t = self.t % T
        # if self.t < 5: # Approach cutting board 

        #     pd, vd = goto(self.t, 5.0, self.p0, self.pboard)
            
        #     #pd = self.p0 + (self.pboard - self.p0) * s0
        #     #vd = (self.pboard - self.p0) * s0dot
        #     Rd = self.R0
        #     wd = np.zeros(3).reshape(-1,1)
            # pd = self.p0
            # Rd = self.R0
            # vd = np.zeros(3).reshape(-1,1)    
            # Compute the joints.      
            # 
        pd = pxyz(0.2 * np.sin(2*self.t) + 0.6, -0.33, 1.13-0.862)
        vd = pxyz(0.4 * np.cos(2*self.t), 0, 0)
        Rd = Rotz(np.pi/2)
        wd = np.array([0,0,0]).reshape(3,1)

        q    = np.zeros((len(jointnames), 1)) 
        qdot = np.zeros((len(jointnames), 1))

        qlast_rarm = self.q_rarm 
        pdlast = self.pd
        Rdlast = self.Rd
        (ptip, Rtip, Jv, Jw) = self.chain_rarm.fkin(qlast_rarm) 
        
        J = np.vstack((Jv, Jw)) 



        eP = ep(pdlast, ptip) 
        er = eR(Rdlast, Rtip) 
        error = np.vstack((eP,er)) 
        x_dot = np.vstack((vd, wd)) 

        gamma = 0.06
        Jinv = np.linalg.pinv(J) # J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6)) 
        
        qdot_rarm = Jinv @ (x_dot + self.lam * error) 
        q_rarm = qlast_rarm + qdot_rarm * self.dt 
        self.q_rarm = q_rarm 
        self.pd = pd 
        self.Rd = Rd
        
        # Right Arm 
        q[23:] =  q_rarm
        qdot[23:] = qdot_rarm 

        print(q[23:])
        
        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = jointnames                # List of names
        cmdmsg.position     = q.flatten().tolist()      # List of positions
        cmdmsg.velocity     = qdot.flatten().tolist()   # List of velocities
        self.pub.publish(cmdmsg)
            

        # else:
            
        #     # Compute the joints.           
        #     pass
            
            
            #pd, vd = goto(self.t, t-5, self.p0, np.array([0.,0.5,0.]).reshape(3,1))
            
        


      



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
