'''cook.py

   This is a demo for moving/placing an ungrounded robot

   In particular, imagine a humanoid robot.  This moves/rotates the
   pelvis frame relative to the world.

   Node:        /cook
   Broadcast:   'pelvis' w.r.t. 'world'     geometry_msgs/TransformStamped

'''

import rclpy
import numpy as np

from rclpy.node                 import Node
from hw5code.GeneratorNode      import GeneratorNode
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState
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

              'r_arm_elx', 'r_arm_ely',
              'r_arm_shx', 'r_arm_shz',
              'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2']


#
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

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Define the various points.        
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        

        #  # Compute the joints.
        # self.q    = np.zeros((len(jointnames), 1))
        # self.qdot = np.zeros((len(jointnames), 1))

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

        # Compute position/orientation of the pelvis (w.r.t. world).
        ppelvis = pxyz(0.0, 0.5, 1.5 + 0.5 * np.sin(self.t/2))
        Rpelvis = Rotz(np.sin(self.t))
        Tpelvis = T_from_Rp(Rpelvis, ppelvis)
        
        # Build up and send the Pelvis w.r.t. World Transform!
        trans = TransformStamped()
        trans.header.stamp    = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id  = 'pelvis'
        #trans.transform       = Transform_from_T(Tpelvis)
        self.broadcaster.sendTransform(trans)

        # Compute the joints.
        q    = np.zeros((len(jointnames), 1))
        qdot = np.zeros((len(jointnames), 1))

        i_relbow = jointnames.index('r_arm_elx')

        # s    =  np.cos(np.pi/2.5 * (self.t-3))
        # sdot = - np.pi/2.5 * np.sin(np.pi/2.5 * (self.t-3))

        q[i_relbow,0]     = -np.pi/2 - np.pi/12 * np.sin(2*self.t)
        qdot[i_relbow, 0] =  np.pi/4 * np.cos(2*self.t)
        
        # Use the path variables to compute the position trajectory.
        # pd = np.array([-0.3*s    , 0.5, 0.75-0.6*s**2  ]).reshape((3,1))
        # vd = np.array([-0.3*sdot , 0.0,     -1.2*s*sdot]).reshape((3,1))


        # qlast = self.q 
        # pdlast = self.pd
        # Rdlast = self.Rd
        # (ptip, Rtip, Jv, Jw) = self.chain.fkin(qlast) 
        
        # Jbar =np.vstack((Jv, Jw))

        # eP = ep(pdlast, ptip)
        # er = eR(Rdlast, Rtip)
        # error = np.vstack((eP,er))

        # x_dot = np.vstack((vd, wd))


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
    node = DemoNode('cook', 100)
    #node = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
