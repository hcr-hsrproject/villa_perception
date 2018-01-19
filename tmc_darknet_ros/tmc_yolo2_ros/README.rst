Overview
++++++++

提供機能
---------

YOLO ver2を利用した認識を行うROSノードと学習用スクリプトを提供する。

ROSインタフェース
+++++++++++++++++

Nodes
-----

- **yolo2_node** 認識を行うノード

- **yolo2_image_node** 認識結果を画像に描画し出力するノード

Suscribed Topics
^^^^^^^^^^^^^^^^

yolo2_node
----------

- **~image** (:ros:msg:`sensor_msgs/Image`)

RGB画像(encoding: RGB8 / BGR8)

yolo2_image_node
----------------

- **~image** (:ros:msg:`sensor_msgs/Image`)

RGB画像

- **~detections** (:ros:msg:`tmc_yolo2_ros/Detections`)

YOLOの認識結果

Published Topics
^^^^^^^^^^^^^^^^

yolo2_node
----------

- **~detections** (:ros:msg:`tmc_yolo2_ros/Detections`)

YOLOの認識結果(x,y:認識対象の中心座標)

yolo2_image_node
----------------

- **~image_result** (:ros:msg:`sensor_msgs/Image`)

YOLOの認識結果(クラス名と領域)を描画した画像

Parameters
^^^^^^^^^^

yolo2_node
----------

- confidence_threshold(double): 認識結果として採用する信頼度の閾値 [0.0, 1.0] (デフォルト: 0.3)

- nms_threshold(double): Non-Maximum supression処理を行う際の重複領域判定(IoU)の閾値 [0.0, 1.0] (デフォルト: 0.4)

- cfg_path(string): ネットワーク定義ファイル(.cfg)のパス

- weights_path(string): 学習結果の重みファイル(.weights)のパス

- class_names_path(string): 認識するクラス名リストファイル(.names)のパス

学習用スクリプトの使い方
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block::

   $ rosrun tmc_yolo2_ros training [学習に関する設定ファイル(.data)のパス] [ネットワーク定義ファイル(.cfg)のパス] [初期重みファイル(.weights)のパス]
