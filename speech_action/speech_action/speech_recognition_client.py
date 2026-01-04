import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand ##TO DO FIX IMPORT


class SpeechRecognitionClient(Node):
    def __init__(self):
        super().__init__('speech_recognition_client')
        self.get_logger().info('Starting speech recognition client.')
        self.goal_handle = None
        self.action_client = ActionClient(
            self, StringCommand, 'speech_recognition/command')

    def hear(self):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        self.get_logger().info('Sending goal request...')
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
            self.get_logger().info(f'Failure status: {status}')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancel succeeded')
            self.goal_handle = None
        else:
            self.get_logger().info('Cancel failed')

    def cancel(self):
        if self.goal_handle is None:
            self.get_logger().info('No goal to cancel')
            return
        self.get_logger().info('Canceling goal...')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)


def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the SpeechRecognitionClient node
    node = SpeechRecognitionClient()

    # Run rclpy.spin() in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    threading.excepthook = lambda x: ()
    thread.start()

    try:
        while True:
            s = input('> ')
            if s == '':
                node.hear()
            elif s == 'c':
                node.cancel()
            else:
                print('Invalid command. Press Enter to hear or "c" to cancel.')
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
