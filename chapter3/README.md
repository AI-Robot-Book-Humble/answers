# 3章の解答例

- このフォルダの```.py```ファイルを，```~/airobot_ws/src/chapter3/speech_action/speech_action```にコピーする．
- ```~/airobot_ws/src/chapter3/speech_action/setup.py```の```entry_points```の設定のリストに以下の行を追加する．
```
            'challenge_3_1 = speech_action.challenge_3_1:main',
```

- 端末で以下を実行
```
cd ~/airobot_ws
source install/setup.bash
```

- まず,2つの端末を開いて,それぞれの端末で音声認識サーバと音声合成サーバを起動します.
```
ros2 run speech_action speech_recognition_server
```
```
ros2 run speech_action speech_synthesis_server
```

- その後，以下のコマンドで```challenge_3_1```を立ち上げます．
```
ros2 run speech_action challenge_3_1
```

