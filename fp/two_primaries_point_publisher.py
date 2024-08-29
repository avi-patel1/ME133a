# Avi Patel and Kemal Pulungan 
# This code makes Atlas run with both the right and left hands acting as the primary task.

'''

   Node:        /cut
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np
import threading

from std_msgs.msg import Float64
from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped, Point, PointStamped
from sensor_msgs.msg            import JointState
from rclpy.qos              import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from KinematicChain     import KinematicChain
from TransformHelpers     import *

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

        # Publisher for intended trajectory marker
        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pub_mark  = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)
        

        ################################################################
        # Kinematic chain and robot initialization.

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
        
        self.chain_larm = KinematicChain(self, 'pelvis', 'l_hand', [ 'back_bkz', 'back_bky', 'back_bkx',
                                                                    'l_arm_shz', 'l_arm_shx',
                                                                    'l_arm_ely', 'l_arm_elx',
                                                                    'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2'])

        
        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Define the various points.        
        self.q0_rarm = np.radians( np.array([0,0,0, 90., 30., 0., -30., 0.,0.,0.]).reshape((-1,1)) )
        (self.p0_rarm, self.R0_rarm, _, _) = self.chain_rarm.fkin(self.q0_rarm)  #np.array( [4., 2., 4.]).reshape((-1,1) )

        self.q0_larm = np.radians( np.array([0,0,0, -90., -30., 0., 60., 0.,0.,0.]).reshape((-1,1)) )
        (self.p0_larm, self.R0_larm, _, _) = self.chain_larm.fkin(self.q0_larm)  #np.array( [4., 2., 4.]).reshape((-1,1) )
        

        self.q_rarm = self.q0_rarm # Right arm
        self.pd_r = self.p0_rarm
        self.Rd_r = self.R0_rarm

        self.q_larm = self.q0_larm # Left arm
        self.pd_l = self.p0_larm
        self.Rd_l = self.R0_larm

        self.q = np.vstack((self.q_rarm, self.q_larm[3:,:]))

        self.lam = 100

        ################################################################
        # Starts the marker points
        # Create the point.
        self.p_right = Point()

        # Create the point message.
        self.point_right = PointStamped()
        self.point_right.header.frame_id = "world"
        self.point_right.header.stamp    = self.get_clock().now().to_msg()
        self.point_right.point           = self.p_right

        # Create the sphere marker.
        self.marker_right = Marker()
        self.marker_right.header.frame_id    = "world"
        self.marker_right.header.stamp       = self.get_clock().now().to_msg()

        self.marker_right.action             = Marker.ADD
        self.marker_right.ns                 = "point"
        self.marker_right.id                 = 1
        self.marker_right.type               = Marker.SPHERE
        self.marker_right.pose.orientation.x = 0.0
        self.marker_right.pose.orientation.y = 0.0
        self.marker_right.pose.orientation.z = 0.0
        self.marker_right.pose.orientation.w = 1.0
        self.marker_right.pose.position      = self.p_right
        self.marker_right.scale.x            = 0.2
        self.marker_right.scale.y            = 0.2
        self.marker_right.scale.z            = 0.2
        self.marker_right.color.r            = 0.0
        self.marker_right.color.g            = 1.0
        self.marker_right.color.b            = 0.0
        self.marker_right.color.a            = 0.8     # Make transparent!

        # # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker_right)

        # Create the point.
        self.p_left = Point()
        self.setvalue()

        # Create the point message.
        self.point_left = PointStamped()
        self.point_left.header.frame_id = "world"
        self.point_left.header.stamp    = self.get_clock().now().to_msg()
        self.point_left.point           = self.p_right

        # Create the sphere marker.
        self.marker_left = Marker()
        self.marker_left.header.frame_id    = "world"
        self.marker_left.header.stamp       = self.get_clock().now().to_msg()

        self.marker_left.action             = Marker.ADD
        self.marker_left.ns                 = "point"
        self.marker_left.id                 = 2
        self.marker_left.type               = Marker.SPHERE
        self.marker_left.pose.orientation.x = 0.0
        self.marker_left.pose.orientation.y = 0.0
        self.marker_left.pose.orientation.z = 0.0
        self.marker_left.pose.orientation.w = 1.0
        self.marker_left.pose.position      = self.p_left
        self.marker_left.scale.x            = 0.2
        self.marker_left.scale.y            = 0.2
        self.marker_left.scale.z            = 0.2
        self.marker_left.color.r            = 1.0
        self.marker_left.color.g            = 0.0
        self.marker_left.color.b            = 0.0
        self.marker_left.color.a            = 0.8     # Make transparent!

        # Create the marker array message.
        self.mark.markers.append(self.marker_left)


        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update) 
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
        
    def setvalue(self):
        # Sets the marker values to the desired position of the right and left hands
        # Set the value.
        ppelvis = pxyz(0.0, 0.0, 0.862)
        self.p_right.x = self.pd_r[0,0] + ppelvis[0,0]
        self.p_right.y = self.pd_r[1,0] + ppelvis[1,0]
        self.p_right.z = self.pd_r[2,0] + ppelvis[2,0]

        self.p_left.x = self.pd_l[0,0] + ppelvis[0,0]
        self.p_left.y = self.pd_l[1,0] + ppelvis[1,0]
        self.p_left.z = self.pd_l[2,0] + ppelvis[2,0]

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
        pd_r =  pxyz(0.1 * np.sin(3*self.t) + 0.6, -0.33, 0.268) # subtract from pelvis
        vd_r =  pxyz(0.3 * np.cos(3*self.t), 0, 0)
        Rd_r = Rotz(pi/2)
        wd_r = np.array([0,0,0]).reshape(3,1)


        qlast_r = self.q[:10] 
        pdlast_r = self.pd_r
        Rdlast_r = self.Rd_r
        (ptip_r, Rtip_r, Jv_r, Jw_r) = self.chain_rarm.fkin(qlast_r)  # Right Arm forward kinematics

        eP_r = ep(pdlast_r, ptip_r) 
        er_r = eR(Rdlast_r, Rtip_r) 
        error_p = np.vstack((eP_r,er_r)) 
        xdot_p = np.vstack((vd_r, wd_r)) 

        J_p =  np.vstack((Jv_r, Jw_r)) 
        
        # Trajectory of left arm (spline motion)
        # We define a general path variable
        sp    =   sin(0.2 * self.t) 
        spdot = 0.2 * cos(0.2 * self.t)

        pf = pxyz(-0.3777-0.5, 0.6200, 1.29068) - ppelvis # final position vector of left arm
        pd_l = 0.5 * (self.p0_larm + pf)  + 0.5 * (pf - self.p0_larm) * sp
        vd_l = 0.5 * (pf - self.p0_larm) * spdot
        # # '''
       
        Rd_l = self.R0_larm 
        wd_l = np.array([0,0,0]).reshape(3,1)

        
        qlast_l = np.vstack(( self.q[0:3,:],self.q[10:17,:]))
        pdlast_l = self.pd_l
        Rdlast_l = self.Rd_l
        (ptip_l, Rtip_l, Jv_l, Jw_l) = self.chain_larm.fkin(qlast_l) # Left Arm forward kinematics
        eP_l = ep(pdlast_l, ptip_l) 
        er_l = eR(Rdlast_l, Rtip_l) 
        error_s = np.vstack((eP_l, er_l)) 
        xdot_s = np.vstack((vd_l, wd_l)) 
        
        J_s = np.vstack((Jv_l, Jw_l)) 

        J_top = np.hstack((J_p, np.zeros((6,7))))
        J_bottom = np.hstack((J_s[:,:3], np.zeros((6,7)), J_s[:,3:]))
        J = np.vstack((J_top, J_bottom))

        W_inv = np.diag([0.1, 0.1, 0.1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

        J_W = J @ W_inv
        J_W_transpose = np.transpose(J_W)

        gamma = 0.5
        J_W_inv = J_W_transpose @ np.linalg.pinv(J_W @ J_W_transpose + gamma**2 * np.eye(12))

        error = np.vstack((error_p, error_s))
        xdot = np.vstack((xdot_p, xdot_s))

        qdot_p = (W_inv @ J_W_inv) @ (xdot + self.lam * error)

        qlast = np.vstack((qlast_r, qlast_l[3:,:]))   

        #Sets the new desired position for the markers
        self.setvalue()


        self.q = qlast + qdot_p * self.dt  
        self.pd_r = pd_r
        self.Rd_r = Rd_r
        self.pd_l = pd_l
        self.Rd_l = Rd_l

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

        self.marker_right.header.stamp = self.now().to_msg()
        self.marker_left.header.stamp = self.now().to_msg()
        self.pub_mark.publish(self.mark)

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

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
