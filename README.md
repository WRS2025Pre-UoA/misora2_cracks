# misora2_cracks
## 内容
 - 受け取った画像からテストピース(金属板)の検出
 - テストピースからクラック(ひび)を検出し、以下の３つをオペレータPCへ渡す
    - クラックの長さ[mm]
    - クラックの幅[mm]
    - 証拠画像：ボックス付き画像（長さ、幅を書き込む）

## コード(src/)
 - detction.cpp : Yolo8を用いたテストピース自動検出を行う
 - detction_main.cpp : detection.cppを単体で起動
 - size.cpp : LSD(Line Segment Detection)を用いてクラックのサイズを測定
 - size_main.cpp : size.cppを単体で起動
 - lsd.c : LSDを行うために必要な関数をもつライブラリ
 - cracks_component.cpp : ノード間の通信を行う ライブラリ化されている
 - cracks_node.cpp : cracks_componentを単体(ノードとして)で起動　あらかじめ読み込む画像のパスを宣言

## 実行コード
### ノード単体で実行
~~~bash!
colcon build 
source install/setup.bash
ros2 run misora2_cracks cracks_node
~~~

### C++プログラム実行
#### detection.cppの場合
~~~bash!
colcon build --symlink-install
./build/misora2_cracks/cracks_detection <画像パス> # テスト画像 : src/misora2_cracks/test1.png
~~~
#### size.cppの場合
~~~bash!
colcon build --symlink-install
./build/misora2_cracks/cracks_size <画像パス> # テスト画像 : src/misora2_cracks/test2.png
~~~