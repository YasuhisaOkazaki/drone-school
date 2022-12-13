# drone-school

### 概要
プログラム内辞書型配列に登録された相対距離（東西南北：nメートル）へ移動します。

接続 -> ARM -> 離陸 -> 辞書型配列にある相対距離へ移動 -> すべてのポイントが終了 -> RTL

dk_homework : 動作切替時の座標から次の移動先を算出します

dk_homework_v2 : 離陸又は直前の目標到達地点の座標から次の移動先を算出します


### 前提パッケージ等
dronekit-pyhon が利用できる環境下にcloneしてください。

### 実行前に（設定）
必要に応じ実行ファイル（ dk_homework.py　又は　dk_homework_v2.py ）の

10行目（connect_msg）～　30行目（wait_time）までを環境に合わせて変更ください。

また、辞書型配列dict_positionの値に応じ　必要があれば targetDistance_mul、stop_groundspeed、stop_timeの値を変更ください。

※変数・配列の説明は、それぞれの上部コメントに記載してあります。

### 実行
SITL等を起動後、実行ファイル（ dk_homework.py　又は　dk_homework_v2.py ）を実行ください
