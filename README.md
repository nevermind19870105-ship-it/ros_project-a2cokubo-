# ROS/Gazebo Visual Marker Demo

このリポジトリには、ROS Noetic と Gazebo を使ったモデル実験に加えて、**緑色のデモマーカーをカメラ画像から認識し、現在位置と短時間先の線形予測位置を画像上に表示する非作動型デモ**を追加しています。

予測は学習モデルと比較するための画像上の推定です。ジンバルデモでは、カメラがマーカーを追従し、別ウィンドウに関節角・カメラ映像・Gazeboの実座標・画像から推定した予測座標を表示します。発射・弾丸機能はありません。

## 必要環境

- Docker / Docker Compose
- リポジトリの Dockerfile が構築する ROS Noetic + Gazebo 環境
- GUIを表示するためのホスト側ディスプレイ設定

## 起動手順

リポジトリのルートでコンテナを構築・起動します。

```bash
docker compose up -d --build
docker exec -it ros_container bash
```

コンテナ内で catkin ワークスペースをビルドします。

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/catkin_ws
catkin_make
source devel/setup.bash
```

続いてデモを起動します。

```bash
roslaunch aiming_robot vision_marker_tracking_demo.launch
```

既定では、合成した滑らかな軌道で事前学習した小型MLPを使います。学習し直す場合はコンテナ内で次を実行します。

```bash
rosrun aiming_robot train_trajectory_predictor.py \
  --output /workspace/catkin_ws/src/aiming_robot/models/trajectory_mlp.npz
```

同じデモで、事前学習MLPと線形外挿を切り替えて比較できます。

```bash
roslaunch aiming_robot vision_marker_tracking_demo.launch predictor_mode:=mlp
roslaunch aiming_robot vision_marker_tracking_demo.launch predictor_mode:=linear
```

この起動ファイルはカメラ映像上に緑色マーカーを検出・表示するデモです。既存のドローン／砲台モデルの起動には `roslaunch aiming_robot spawn_models.launch` を使用します。

別ターミナルでコンテナに入り、注釈付き画像を表示します。

```bash
rosrun image_view image_view image:=/vision/annotated_image
```

## カメラジンバル追従デモ

砲台形モデルのパン／チルト関節をカメラ台として使い、画像中心からの緑色マーカーのずれをもとに、Gazebo内でカメラの向きを調整します。GazeboのGUIで実行する場合は次を起動してください。

```bash
roslaunch aiming_robot vision_gimbal_demo.launch
```

このデモは緑色マーカーの現在位置をカメラ中央へ戻すシミュレーション専用のジンバル動作です。未来位置は表示と座標比較に使います。発射・弾丸機能はなく、パン／チルト指令は関節範囲内に制限します。

起動すると、Gazeboとは別に「Gazebo Gimbal Monitor」ウィンドウが開きます。左側にカメラ映像、右側にパン／チルト関節角、Gazeboから取得したマーカーの実座標、画像予測点から逆投影した予測座標を表示します。予測座標の奥行きは、マーカーを直径0.30 mの球として見かけの幅から推定します。したがって実座標はGazeboの正解値、予測座標は画像からの推定値です。

## トピック

| トピック | 内容 |
|---|---|
| `/camera/image_raw` | Gazeboカメラの入力画像 |
| `/camera/camera_info` | Gazeboカメラのカメラ情報 |
| `/vision/annotated_image` | 緑色マーカーの現在位置と線形予測位置を重ねた画像 |
| `/vision/predicted_pixel` | 予測画像座標(u,v)と見かけのマーカー幅(px) |

## 認識パラメータ

`vision_marker_tracker.py` はHSV色空間で緑色領域を抽出します。デフォルト値は次のとおりです。

- `~hsv_lower`: `[40, 70, 70]`
- `~hsv_upper`: `[85, 255, 255]`
- `~min_area`: `100.0`（最小輪郭面積）
- `~prediction_horizon`: `0.25`（予測時間、秒）
- `~history_length`: `8`（速度推定に使う検出点数）

カメラの露出・照明・描画色によってはパラメータ調整が必要です。

## 実装上の範囲

- カメラ画像の購読
- OpenCVによる緑色マーカー領域の抽出
- 最近の検出履歴による速度推定と短時間先の位置予測
- 現在位置・予測位置を重ねた画像の配信
- 予測画像点の配信と、別ウィンドウでの関節角・映像・実座標・推定予測座標の表示
- Gazebo内でマーカーをゆっくり移動させるデモ

MLPは合成軌道180本で事前学習し、別に生成した12本の軌道で評価しています。固定シードでの640×480換算の平均位置誤差はMLP 4.30 px、線形外挿 4.42 pxでした。この差は小さく、合成軌道上の結果です。実際のGazebo映像から取った軌道での再学習・比較が次の改善点です。予測は画像注釈だけに使い、ROSの座標出力や砲台制御には接続しません。ROS/Gazebo上での実行とGUI表示は未確認です。環境によるカメラ向き、Gazeboプラグイン、ディスプレイ設定の調整が必要になる場合があります。
