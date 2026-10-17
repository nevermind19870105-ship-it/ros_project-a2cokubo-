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

砲台形モデルのパン／チルト関節をカメラ台として使い、左右FMCWレーダーから予測したworld座標へ向けてGazebo内で関節を調整します。中央カメラの緑色マーカー検出・予測は比較用に並行表示します。GazeboのGUIで実行する場合は次を起動してください。

```bash
roslaunch aiming_robot vision_gimbal_demo.launch
```

このデモはレーダー融合の予測点を砲台関節の目標角へ変換するシミュレーション専用動作です。カメラ画像による予測も独立して実行され、結果を並べて比較できます。発射・弾丸機能はなく、パン／チルト指令は関節範囲内に制限します。

起動すると、Gazeboとは別に「Gazebo Gimbal Monitor」ウィンドウが開きます。左側にカメラ映像、右側にパン／チルト関節角、Gazeboから取得したマーカーの実座標、画像予測点から逆投影した予測座標を表示します。予測座標の奥行きは、マーカーを直径0.30 mの球として見かけの幅から推定します。したがって実座標はGazeboの正解値、予測座標は画像からの推定値です。

## トピック

| トピック | 内容 |
|---|---|
| `/camera/image_raw` | Gazeboカメラの入力画像 |
| `/camera/camera_info` | Gazeboカメラのカメラ情報 |
| `/vision/annotated_image` | 緑色マーカーの現在位置と線形予測位置を重ねた画像 |
| `/vision/predicted_pixel` | 予測画像座標(u,v)と見かけのマーカー幅(px) |
| `/shooter/radar_left/image` | 左FMCWレーダーの距離×角度強度画像（mono8） |
| `/shooter/radar_right/image` | 右FMCWレーダーの距離×角度強度画像（mono8） |
| `/radar/target_position` | 左右レーダー画像から融合した標的位置（world座標、m） |
| `/radar/predicted_target_position` | 直近のレーダー座標履歴から線形外挿した短時間先の位置 |

## 3センサー構成（中央カメラ + 左右FMCWレーダー）

砲台の固定ベース両側の高さ約1.5 mに、CPUで動くFMCWレーダーシミュレーターを搭載しています。これはデモ標的の高さに合わせ、水平走査レーダーの視野に標的が入るようにした配置です。既存の `vision_gimbal_demo.launch` は専用ワールドを使い、左右のレーダー画像を上記トピックへ配信します。起動時に環境ノイズを切って反射を見やすくし、デモ距離に合った計算量へ調整します。中央カメラと画像予測の構成はそのまま利用します。

FMCWのROSパッケージとRmagine/EmbreeはDockerイメージ構築時に固定した上流コミットからビルドされます。CPU版のためGPUは不要ですが、初回の `docker compose up -d --build` は通常より時間がかかります。左右レーダー画像は各1024距離セル × 400角度セル、既定約4 Hzです。`radar_target_tracker.py` は各画像の強い反射を検出し、センサー位置を考慮して左右のworld座標を融合します。3点以上の履歴がそろうと、0.25秒先を線形外挿します。現在のデモはロボットを原点に固定し、高さを走査面の1.50 mに固定する簡易推定です。反射材はシミュレーターの既定値を使い、レーダー機体自身の運動によるドップラー効果も考慮しません。したがって、これは座標推定・予測の動作確認用ベースラインで、学習済みレーダー予測器ではありません。中央カメラによる緑色マーカー検出とMLP予測は別系統で残しています。

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

MLPは合成軌道180本で事前学習し、別に生成した12本の軌道で評価しています。固定シードでの640×480換算の平均位置誤差はMLP 4.30 px、線形外挿 4.42 pxでした。この差は小さく、合成軌道上の結果です。実際のGazebo映像から取った軌道での再学習・比較が次の改善点です。現在の砲台指令はレーダー座標の線形外挿を使用し、画像MLP予測は比較表示用です。レーダー位置推定は原点固定・2D走査面の単純モデルです。レーダー計測に基づく動作であり、実環境でのRFセンシング性能を表すものではありません。発射・弾丸機能はありません。
