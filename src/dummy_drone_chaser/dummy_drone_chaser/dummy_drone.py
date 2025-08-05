import rclpy, math
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from rclpy.node import Node
from random import uniform


class DummyDrone(Node):

    def __init__(self):
        super().__init__('dynamic_drone_broadcaster')

        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.prev_pos = [0.0]*6
        self.target_pos = DummyDrone.dd_svg()
        self.alpha = 0.01

    
    def timer_cb(self):

        self.prev_pos = [self.prev_pos[i] + self.alpha * (self.target_pos[i] - self.prev_pos[i]) for i in range(3)]
        distance = math.sqrt(
            sum((self.target_pos[i] - self.prev_pos[i])**2 for i in range(3))
        )


        #dtr = [self.target_pos[0] - self.prev_pos[0],
        #                    self.target_pos[1] - self.prev_pos[1],
        #                    self.target_pos[2] - self.prev_pos[2]]
        #
        #distance = math.sqrt(math.pow(dtr[0], 2) + math.pow(dtr[1], 2) + math.pow(dtr[2], 2))
#
        ## Normalize the direction vector to get the unit vector
        #if distance > 0:
        #    dtr = [v/distance for v in dtr]
        #
        ## Move pre_pos a step closer to target_pos
        #self.prev_pos = [self.prev_pos[0] + dtr[0] * self.speed,
        #                 self.prev_pos[1] + dtr[1] * self.speed,
        #                 self.prev_pos[2] + dtr[2] * self.speed]
        
        self.get_logger().info(f"Current position: {self.prev_pos}, Target: {self.target_pos}")
        self.broadcast_transform(self.prev_pos)

        if distance < 0.5:  # reduced threshold for nicer switching
            self.target_pos = DummyDrone.dd_svg()
            self.get_logger().info("New target acquired!")


    def broadcast_transform(self, pos):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'drone'
        t.transform.translation.y = pos[0]
        t.transform.translation.x = pos[1]
        t.transform.translation.z = pos[2]

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)
    
    # dummy drone sim value generator
    @classmethod
    def dd_svg(cls): 
        """
        Use case:
        [x, y, z, r, p, y]: x-axis, y-axis, z-axis, roll, pitch, yaw
        ---
        return a list after generating dummy values for simulation purpose
        """
        x, y, z = uniform(-5, 5), uniform(-5, 5), uniform(0, 5)
        roll, pitch, yaw = uniform(-3.14, 3.14), uniform(-3.14, 3.14), uniform(-3.14, 3.14)

        return [x, y, z, roll, pitch, yaw]



def main():
    rclpy.init()

    node = DummyDrone()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()