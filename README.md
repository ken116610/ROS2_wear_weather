# wearコマンド 気温と服装
![test](https://github.com/ken116610/ROS2_wear_weather/actions/workflows/test.yml/badge.svg)

# 目次
1 コマンド説明\
2 使い方\
3 実行例\
4 動作環境\
5 ライセンス

## コマンド説明
今日の気温を入力すると気温に合った服装を選んでくれるコマンド

## 使い方
### 1.リポジトリのインストール
```
$ git clone https://github.com/ken116610/ROS2_wear_weather.git
$ cd ros2_ws
$ coicon build --symlink-install
$ source install/setup.bash
```

### 2.コマンドの実行
3つのターミナルを使用します。
#### 1つ目ノードの起動
```
$ ros2 run wear_advisor wear_node
```
#### 2つ目結果の表示
```
$ ros2 run wear_advisor wear_print
```
#### 3つ目気温の入力と送信
weatherのところに気温を入力する
```
$ echo "weather" | ros2 run wear_advisor wear_pub
```

## 実行例
```
$ echo "15" | ros2 run wear_advisor wear_pub
トップス：長袖
ボトムス：長ズボン
アウター：薄手ジャケット
```

## 動作環境
Ubuntu 22.04 LTS\
Python3 系\
GitHub Actions\
ROS2


## ライセンス
・本コマンドは三条項BSDライセンスの下にて、使用および複製が許可されています>。\
・© 2025 Yuken Ro
