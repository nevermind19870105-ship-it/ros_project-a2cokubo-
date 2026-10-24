# ROS/Gazebo Visual Marker Demo

このリポジトリには、ROS Noetic と Gazebo を使ったモデル実験に加えて、**緑色のデモマーカーをカメラ画像から認識し、現在位置と短時間先の線形予測位置を画像上に表示する非作動型デモ**を追加しています。

予測は学習モデルと比較するための画像上の推定です。ジンバルデモでは、カメラがマーカーを追従し、別ウィンドウに関節角・カメラ映像・Gazeboの実座標・画像から推定した予測座標・単一レーダーの推定位置とMLP予測位置・レーダー画像を表示します。これは標的位置と照準用の予測位置の表示であり、発射・弾丸機能はありません。

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

砲台形モデルのパン／チルト関節をカメラ台として使い、カメラで検出した緑色マーカーをGazebo内で追従します。単一FMCWレーダーの距離・方位推定と予測も比較用に並行表示します。GazeboのGUIで実行する場合は次を起動してください。

```bash
roslaunch aiming_robot vision_gimbal_demo.launch
```

このデモはカメラ画像の追跡誤差を砲台関節の目標角へ変換するシミュレーション専用動作です。レーダー画像と推定位置も別ウィンドウで確認できます。発射・弾丸機能はなく、パン／チルト指令は関節範囲内に制限します。

起動すると、Gazeboとは別に「Gazebo Gimbal Monitor」ウィンドウが開きます。左側にカメラ映像、右側にパン／チルト関節角、Gazeboから取得したマーカーの実座標、画像予測点から逆投影した予測座標を表示します。下部にはレーダー反射強度の相対ヒートマップと、距離ごとの最大反射強度を示す波形パネルを表示します。波形パネルの横軸は距離[m]、縦軸はセンサー出力の強度値で、見やすさのため縦軸上限はスキャンごとに調整されます。予測座標の奥行きは、マーカーを直径0.30 mの球として見かけの幅から推定します。したがって実座標はGazeboの正解値、予測座標は画像からの推定値です。

Gazebo上では、チルト台のカメラ本体を青色、砲台から左側へ離した地上支柱上のレーダー本体を赤色で表示します。レーダー本体上の矢印は、センサーの前方基準であるローカル+X（方位0°）を示します。レーダーは水平360°を走査します。

## トピック

| トピック | 内容 |
|---|---|
| `/camera/image_raw` | Gazeboカメラの入力画像 |
| `/camera/camera_info` | Gazeboカメラのカメラ情報 |
| `/vision/annotated_image` | 緑色マーカーの現在位置と線形予測位置を重ねた画像 |
| `/vision/predicted_pixel` | 予測画像座標(u,v)と見かけのマーカー幅(px) |
| `/shooter/radar_left/image` | FMCWレーダーの距離×角度強度画像（mono8） |
| `/radar/target_position` | レーダーから推定した標的位置（world座標、m） |
| `/radar/predicted_target_position` | 事前学習MLP（または比較用線形外挿）が予測した短時間先の位置 |

## 2センサー構成（カメラ + FMCWレーダー）

砲台から左側へ1.5 m離した地上支持マストの上部（高さ約1.5 m）に、CPUで動くFMCWレーダーシミュレーターを1基設置しています。センサーの走査面はデモ標的の高さに合わせ、水平に保っています。`vision_gimbal_demo.launch` は専用ワールドを使い、レーダー画像を上記トピックへ配信します。起動時に環境ノイズを切って反射を見やすくし、デモ距離に合った計算量へ調整します。カメラと画像予測の構成はそのまま利用します。

FMCWのROSパッケージとRmagine/EmbreeはDockerイメージ構築時に固定した上流コミットからビルドされます。CPU版のためGPUは不要ですが、初回の `docker compose up -d --build` は通常より時間がかかります。レーダー画像は1024距離セル × 400角度セル、既定約4 Hzです。`radar_target_tracker.py` は前方セクターの強い反射を検出し、レーダー位置を考慮してworld座標へ変換します。8点（約2秒）のレーダー座標履歴を4 HzにそろえてMLPへ入力し、0.25秒先のXY座標を推定します。モデルがない場合、または履歴が足りない場合は線形外挿へ切り替わります。現在のデモはロボットを原点に固定し、高さを走査面の1.50 mに固定する簡易推定です。反射材はシミュレーターの既定値を使い、レーダー機体自身の運動によるドップラー効果も考慮しません。

レーダー予測モデルは、シミュレーターを動かさなくても合成した曲線・旋回軌道から事前学習できます。履歴8点のXY変位を入力、次の1点（0.25秒後）の変位を正解として学習します。各観測にはレーダー座標誤差を模したノイズを加え、軌道単位で分けた評価データ上の誤差を線形外挿と比較してから `models/radar_trajectory_mlp.npz` に保存します。現在の固定シード設定では、未学習の合成軌道における平均誤差はMLP 0.058 m、線形外挿0.075 mでした。この数値は合成データ上の参考値です。

```bash
rosrun aiming_robot train_radar_predictor.py
```

デモではカメラの視覚追跡からパン／チルトを制御し、バルーンを継続して画面中央へ追従させます。レーダーは距離・方位画像から位置推定と予測を行い、モニター画面で比較できます。検出角度はセンサーの `-pi` 始まりに合わせ、後方・側方の反射を除外します。単一レーダーのため複数センサー間の合意判定は行いません。ターゲット反射が弱い場合は有効な位置が得られないため、照準にはカメラ追跡を使います。

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

カメラMLPは合成画像軌道180本で事前学習し、別の12本で評価します。レーダーMLPも合成レーダー座標軌道で学習し、未学習の軌道に対する誤差を線形外挿と比べます。どちらも現在は合成データによる評価で、実測性能を示すものではありません。レーダー位置推定は原点固定・2D走査面の単純モデルです。発射・弾丸機能はありません。
