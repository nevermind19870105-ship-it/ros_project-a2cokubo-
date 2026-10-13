# ROS/Gazebo Visual Marker Demo

このリポジトリには、ROS Noetic と Gazebo を使ったモデル実験に加えて、**緑色のデモマーカーをカメラ画像から認識し、検出領域を画像上に表示する非作動型のコンピュータビジョン・デモ**を追加しています。

このデモは画像への注釈表示のみを行います。未来位置の予測、照準・パン／チルト制御、発射・アクチュエータ出力は行いません。

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

この起動ファイルはカメラ映像上に緑色マーカーを検出・表示するデモです。既存のドローン／砲台モデルの起動には `roslaunch aiming_robot spawn_models.launch` を使用します。

別ターミナルでコンテナに入り、注釈付き画像を表示します。

```bash
rosrun image_view image_view image:=/vision/annotated_image
```

## トピック

| トピック | 内容 |
|---|---|
| `/camera/image_raw` | Gazeboカメラの入力画像 |
| `/camera/camera_info` | Gazeboカメラのカメラ情報 |
| `/vision/annotated_image` | 緑色マーカーの検出枠を描画した画像 |

## 認識パラメータ

`vision_marker_tracker.py` はHSV色空間で緑色領域を抽出します。デフォルト値は次のとおりです。

- `~hsv_lower`: `[40, 70, 70]`
- `~hsv_upper`: `[85, 255, 255]`
- `~min_area`: `100.0`（最小輪郭面積）

カメラの露出・照明・描画色によってはパラメータ調整が必要です。

## 実装上の範囲

- カメラ画像の購読
- OpenCVによる緑色マーカー領域の抽出
- 検出領域を重ねた画像の配信
- Gazebo内でマーカーをゆっくり移動させるデモ

現段階では、ROS/Gazebo上での実機動作・GUI表示を実行して確認したものではありません。環境によるカメラ向き、Gazeboプラグイン、ディスプレイ設定の調整が必要になる場合があります。
