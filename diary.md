2018.7.19
環境構築をした。Gazeboで動作したが遅い。
０．２倍程度の速度になっている。問題はないがおそい。

2018.7.22
トピックの確認をした。
バンパーのトピックがない？確認する。
us_left,us_rightが超音波センサーのようだが、１秒毎にしかトピックが出ていない。
scanがLiDAR。これは0.1sごとにでている。
scanはRVIZで視覚化ができた。 
 1.Fixed Frame -> red_bot/laser
 2.ADD  ->  Topic / red_bot/scan

結構大変そうだ。

ROSを使うなら既存のパッケージを活用する方向性が正しいか。

とりあえずキーボードで動くようにしたい。
TurtlesimのTeleopで、名前空間だけ変えればいけるのではないだろうか


2018.7.23
バンパーのトピックはbumperらしい。一回も発行されていないトピックはrostopic listで出てこないようだ。

telop_keyでコントロールできるようにした。
使いにくかったので、yocs_keyopを入れた。

	
> rosrun yocs_keyop yocs_keyop /yocs_keyop/cmd_vel:=/red_bot/cmd_vel

2018.07.25
ARCOをが立ち上がらないLaunchをつくった
runAlongTheWall.pyを作成

LiDarのSample Code
https://answers.ros.org/question/57841/ros-hokuyo_node-laserscan-subscriber-msg-data-class-python/

data.rangesに距離が入っている
０と３５９が正面。９０が左側面
０．２５距離ですれすれ。

ros-kinetic-pid　を入れた
http://wiki.ros.org/pid


2018.7.30
左側の壁までの距離のみをPubするNodeをつくった
Launchファイルに追記した


2018.07.31
PIDノードから受け取ってcmd_velに出すノードをつくった
とりあえずPIDで壁沿いに沿うことはできそう

2018.8.1
壁との相対角度を正しく算出したい　ー＞OK
壁からの距離を正しく計測できるようにしたい。

2018.8.5
とりあえずシーケンスで動くプログラムができた。２点取れる

2018.8.6
３点取れるようになった。
盲目的に角度変更するプログラムが必要だ。

2018.8.7
とりあえずDeploy

