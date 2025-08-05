from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler
from tf2_ros.transform_broadcaster import TransformBroadcaster
from random import uniform
import rclpy.time
import math




class DroneChaser(Node):

    def __init__(self):
        super().__init__('drone_chaser')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_cb)  # Update every 0.5 seconds

        self.curr_pos = DroneChaser.dd_svg()
        self.alpha = 0.01
    
    def timer_cb(self):
        
        # getting the current position of drone.
        from_frame_rel = 'map'
        to_frame_rel = 'drone'

        try:
            t = self.buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f"Could not transform {to_frame_rel} to {from_frame_rel}: {e}")
            return
    
        msg = Twist()
        msg.linear.x = t.transform.translation.x
        msg.linear.y = t.transform.translation.y
        msg.linear.z = t.transform.translation.z

        direc_vctr = [
                msg.linear.x - self.curr_pos[0],
                msg.linear.y - self.curr_pos[1],
                msg.linear.z - self.curr_pos[2]]
        
        dist = math.sqrt(direc_vctr[0]**2 + direc_vctr[1]**2 + direc_vctr[2]**2)
        if dist > 0:
            direc_vctr[0] /= dist
            direc_vctr[1] /= dist
            direc_vctr[2] /= dist
        
        self.curr_pos[0] = (1 - self.alpha) * self.curr_pos[0] + msg.linear.x * self.alpha
        self.curr_pos[1] = (1 - self.alpha) * self.curr_pos[1] + msg.linear.y * self.alpha
        self.curr_pos[2] = (1 - self.alpha) * self.curr_pos[2] + msg.linear.z * self.alpha
        
        self.get_logger().info(f"A: >> X: {self.curr_pos[0]}, Y: {self.curr_pos[1]}, Z: {self.curr_pos[2]}")

        t1 = TransformStamped()

        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'chaser'

        #1 Set the position of the drone (x, y, z)
        t1.transform.translation.x = self.curr_pos[0]
        t1.transform.translation.y = self.curr_pos[1]
        t1.transform.translation.z = self.curr_pos[2]

        #1 Set the rotation (you can modify this if you want to simulate rotation as well)
        q1 = quaternion_from_euler(0.0, 0.0, 0.0)  # Assuming no rotation for simplicity
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(t1)

    @classmethod
    def dd_svg(cls):
        # Generate random coordinates for target drone
        x, y, z = uniform(1, 10), uniform(1, 10), uniform(1, 10)
        return [x, y, z]


def main(args=None):
    import rclpy
    rclpy.init(args=args)

    drone_chaser = DroneChaser()

    rclpy.spin(drone_chaser)

    drone_chaser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
