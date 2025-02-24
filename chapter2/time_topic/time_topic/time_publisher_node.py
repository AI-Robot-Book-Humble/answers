import rclpy                         # ROS2のPythonモジュール
from rclpy.node import Node          # rclpy.nodeモジュールからNodeクラスをインポート
from std_msgs.msg import String      # std_msgs.msgモジュールからStringクラスをインポート
import datetime                      # 日時取得するためのライブライをインポート
import pytz                          # 日本時間に対応するためにタイムゾーンのライブラリをインポート


class TimePublisher(Node):  #"日本時間を1秒毎にパブリッシュするクラス
    def __init__(self):     # コンストラクタ
        super().__init__('time_publisher_node')
        self.pub = self.create_publisher(String, 'topic', 10)   # パブリッシャの生成
        self.timer = self.create_timer(1, self.timer_callback)  # タイマーの生成
        self.jst = pytz.timezone('Asia/Tokyo')                  # タイムゾーンJSTの設定
 
    def timer_callback(self):  # コールバック関数
        msg = String()
        jst_time = datetime.datetime.now(self.jst).strftime('%Y-%m-%d %H:%M:%S')  # JST現在時刻を取得し文字列に変換
        msg.data = jst_time
        self.pub.publish(msg)
        self.get_logger().info(f'パブリッシュ: {msg.data}')


def main(args=None):  # main関数
    rclpy.init()
    node = TimePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
