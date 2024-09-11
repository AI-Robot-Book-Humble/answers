# 第6章のチャレンジの解答例

- このフォルダの`.py` ファイルを，`~/airobot_ws/src/chapter6/crane_plus_commander/crane_plus_commander` にコピーする．
- `~/airobot_ws/src/chapter6/crane_plus_commander/setup.py` の `entry_points`の設定のリストに以下の行を追加する．
  ```
            'challenge6_1 = crane_plus_commander.challenge6_1:main',
            'challenge6_2 = crane_plus_commander.challenge6_2:main',
            'challenge6_4 = crane_plus_commander.challenge6_4:main',
            'challenge6_5 = crane_plus_commander.challenge6_5:main',
            'challenge6_6 = crane_plus_commander.challenge6_6:main',
            'challenge6_7 = crane_plus_commander.challenge6_7:main',
            'challenge6_8 = crane_plus_commander.challenge6_8:main',
  ```
- 端末で以下を実行
  ```
  cd ~/airobot_ws
  colcon build --packages-select crane_plus_commander
  source install/setup.bash
  ```
- CRANE+の実機かシミュレータのノード群をローンチした後に，端末で以下を実行（X = 1,2,4,5,6,7,8）
  ```
  ros2 run crane_plus_commander challenge6_X
  ```