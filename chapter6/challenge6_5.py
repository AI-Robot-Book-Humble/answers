import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
from math import sqrt
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import (
    inverse_kinematics, from_gripper_ratio, joint_in_range, GRIPPER_MAX, GRIPPER_MIN)


# tfのフレームで与えられた点へCRANE+ V2の手先を位置決めするノード
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.publisher_joint = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory', 10)
        self.publisher_gripper = self.create_publisher(
            JointTrajectory,
            'crane_plus_gripper_controller/joint_trajectory', 10)

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_joint(self, q, time):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_joint.publish(msg)

    def publish_gripper(self, gripper, time):
        msg = JointTrajectory()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_gripper.publish(msg)

    def get_frame_position(self, frame_id):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'crane_plus_base',
                frame_id,
                when,
                timeout=Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return None
        t = trans.transform.translation
        r = trans.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
        return [t.x, t.y, t.z, roll, pitch, yaw]


# リストで表された3次元座標間の距離を計算する
def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

def main():
    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    commander = Commander()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    threading.excepthook = lambda x: ()
    thread.start()

    # 最初の指令をパブリッシュする前に少し待つ
    time.sleep(2.0)

    # 初期ポーズへゆっくり移動させる
    joint = [0.0, -1.16, -2.01, -0.73]
    gripper = GRIPPER_MIN
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # 逆運動学の解の種類
    elbow_up = True

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    # 状態
    INIT = 0
    WAIT = 1
    DONE = 2
    state = INIT

    print('rキーを押して再初期化')
    print('Escキーを押して終了')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            time.sleep(0.01)            
            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                if c == 'r':
                    print('再初期化')
                    state = INIT
                elif ord(c) == 27:  # Escキー
                    break

            position = commander.get_frame_position('target')
            if position is None:
                print('対象のフレームが見つからない')
            else:
                xyz_now = position[0:3]
                time_now = time.time()
                if state == INIT:
                    xyz_first = xyz_now
                    time_first = time_now
                    state = WAIT
                elif state == WAIT:
                    if dist(xyz_now, xyz_first) > 0.01:
                        state = INIT
                    elif time_now - time_first > 1.0:
                        state = DONE
                        # 把持ポーズの計算
                        [x2, y2, z2] = xyz_now
                        pitch = 0.0
                        elbow_up = True
                        joint2 = inverse_kinematics([x2, y2, z2, pitch], elbow_up)
                        if joint2 is None:
                            print('把持ポーズの逆運動学計算に失敗')
                            continue
                        if not all(joint_in_range(joint2)):
                            print('把持ポーズが可動範囲外')
                            continue
                        # 準備ポーズの計算
                        offset = 0.05
                        d = sqrt(x2**2 + y2**2)
                        x1 = x2 - offset * x2 / d
                        y1 = y2 - offset * y2 / d
                        z1 = z2
                        joint1 = inverse_kinematics([x1, y1, z1, pitch], elbow_up)
                        if joint1 is None:
                            print('準備ポーズの逆運動学計算に失敗')
                            continue
                        if not all(joint_in_range(joint1)):
                            print('準備ポーズが可動範囲外')
                            continue
                        # グリッパを開く
                        dt = 1.0
                        gripper_ratio = 0.0
                        client_result = commander.publish_gripper(from_gripper_ratio(gripper_ratio), dt)
                        time.sleep(dt)
                        # 準備ポーズへ動く
                        dt = 3.0
                        client_result = commander.publish_joint(joint1, dt)
                        time.sleep(dt)
                        # 把持ポーズへ動く
                        dt = 1.0
                        client_result = commander.publish_joint(joint2, dt)
                        time.sleep(dt)
                        # グリッパを閉じる
                        dt = 1.0
                        gripper_ratio = 0.95
                        client_result = commander.publish_gripper(from_gripper_ratio(gripper_ratio), dt)
                        time.sleep(dt)
                        # 運搬ポーズへ動く
                        joint = [-0.00, -1.37, -2.52, 1.17]
                        dt = 3.0
                        client_result = commander.publish_joint(joint, dt)
                        time.sleep(dt)
                        print('完了')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # 終了ポーズへゆっくり移動させる
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = GRIPPER_MAX
        dt = 5
        commander.publish_joint(joint, dt)
        commander.publish_gripper(gripper, dt)

    rclpy.try_shutdown()
