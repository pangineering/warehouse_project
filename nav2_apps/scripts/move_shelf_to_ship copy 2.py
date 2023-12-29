import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def execute(self):
        self.send_goal()

    def send_goal(self):
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = 5.43399
        goal_pose.pose.pose.position.y = -0.060445
        goal_pose.pose.pose.position.z = 0.0

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client.execute())


if __name__ == '__main__':
    main()
