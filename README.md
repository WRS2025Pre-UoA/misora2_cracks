# misora2_cracks
## 内容
 - 受け取った画像からテストピース(金属板)の検出
 - テストピースからクラック(ひび)を検出し、以下の３つをオペレータPCへ渡す
    - クラックの長さ[mm]
    - クラックの幅[mm]
    - 証拠画像：ボックス付き画像（長さ、幅を書き込む）
## クローン方法
~~~bash!
cd ワークスペース/src
git clone git@github.com:WRS2025Pre-UoA/misora2_cracks.git
~~~
## コード(src/)
 - detction.cpp : Yolo8を用いたテストピース自動検出を行う
 - detction_main.cpp : detection.cppを単体で起動
 ~~~bash!
 ros2 run misora2_cracks cracks_detection
 ~~~
 - cracks_component.cpp : ノード間の通信を行う ライブラリ化されている
 - cracks_node.cpp : cracks_componentを単体(ノードとして)で起動　あらかじめ読み込む画像のパスを宣言
 ~~~bash!
 ros2 run misora2_cracks cracks_node
 ~~~
