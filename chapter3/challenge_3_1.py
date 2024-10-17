from threading import Thread, Event
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand

"""
チャレンジ3.1の回答例

オウム返しのプログラムを改良
【必要なプログラム】
・challenge_3_1.py              speech_client.pyを一部変更
・speech_recognition_server.py  変更なし
・speech_synthesis_server.py    変更なし

新たなプログラムはsetup.pyに追記が必要です。
speech_client.pyをchallenge_3_1_1.pyに変更して、
オウム返しと同じ順序でプログラムを起動してください

"""


class StringCommandActionClient:
    def __init__(self, node, name):
        self.name = name
        self.logger = node.get_logger()
        self.action_client = ActionClient(node, StringCommand, name)
        self.event = Event()

    def send_goal(self, command: str):
        self.logger.info(f'{self.name} アクションサーバ待機...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = command
        self.logger.info(f'{self.name} ゴール送信... command: \'{command}\'')
        self.event.clear()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.event.wait(20.0)
        if self.action_result is None:
            self.logger.info(f'{self.name} タイムアウト')
            return None
        else:
            result = self.action_result.result
            status = self.action_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info(f'{self.name} 結果: {result.answer}')
                self.goal_handle = None
                return result.answer
            else:
                self.logger.info(f'{self.name} 失敗ステータス: {status}')
                return None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info(f'{self.name} ゴールは拒否されました')
            return
        self.goal_handle = goal_handle
        self.logger.info(f'{self.name} ゴールは受け付けられました')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.event.set()


class SpeechClient(Node):
    def __init__(self):
        super().__init__('speech_client')
        self.get_logger().info('音声対話ノードを起動します．')
        self.recognition_client = StringCommandActionClient(
            self, 'speech_recognition/command')
        self.synthesis_client = StringCommandActionClient(
            self, 'speech_synthesis/command')
        self.thread = Thread(target=self.run)
        self.thread.start()

        # 追加した変数
        self.objects = ['bottle', 'cup']
        self.places = ['kitchen', 'living']

    # 関数の中を一部変更
    def run(self):
        self.running = True
        while self.running:
            self.synthesis_client.send_goal('I\'m ready.')
            self.get_logger().info('I\'m ready.')

            text = None
            while text is None:
                text = self.recognition_client.send_goal('')

            target_object, target_palce = self.search_object_and_place(text)

            self.synthesis_client.send_goal(f'I will go to the {target_palce} and grab a {target_object}')
            self.get_logger().info(f'I will go to the {target_palce} and grab a {target_object}')

    # 追加した関数
    def search_object_and_place(self, text):

        self.get_logger().info(f'受けとったテキスト "{text}"')

        target_object = None
        target_place = None

        for _object in self.objects:
            if _object in text:
                target_object = _object

        for _place in self.places:
            if _place in text:
                target_place = _place

        return target_object, target_place



def main():
    rclpy.init()
    node = SpeechClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        pass

    rclpy.try_shutdown()
