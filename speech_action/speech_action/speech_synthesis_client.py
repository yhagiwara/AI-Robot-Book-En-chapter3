import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand ##TO DO FIX IMPORT


class SpeechSynthesisClient(Node):
    def __init__(self):
        super().__init__('speech_synthesis_client')
        self.get_logger().info('Starting speech synthesis client.')
        self.goal_handle = None
        self.action_client = ActionClient(
            self, StringCommand, 'speech_synthesis/command')

    def say(self, s):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = s
        self.get_logger().info('Sending goal...')
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return
        self.goal_handle = goal_handle
        self.get_logger().info('Goal was accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result: {result.answer}')
            self.goal_handle = None
        else:
            self.get_logger().info(f'Failed status: {status}')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Canellation succeeded')
            self.goal_handle = None
        else:
            self.get_logger().info('Cancellation failed')

    def cancel(self):
        if self.goal_handle is None:
            self.get_logger().info('No goal to cancel')
            return
        self.get_logger().info('Cancellation')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)


def main():
    # Initialize the ROS client
    rclpy.init()

    # Create an instance of the node class
    node = SpeechSynthesisClient()

    # Run rclpy.spin() in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    threading.excepthook = lambda x: ()
    thread.start()

    try:
        while True:
            s = input('> ')
            if s == '':
                node.cancel()
            else:
                node.say(s)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
