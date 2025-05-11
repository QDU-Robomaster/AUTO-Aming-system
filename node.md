# 相机标定
```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py

ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.015 image:=/camera/image camera:=/camera
```
`size` 为棋盘格交点的数量 `width x height`

`square` 为一个黑色棋子的大小，单位为 m

# odom2camera测量，改参数

