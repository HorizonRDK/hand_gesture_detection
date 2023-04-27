# 功能介绍

hand_gesture_detection package是使用hobot_dnn package开发的手势识别算法示例，在地平线X3开发板上使用手势识别模型和人手检测数据利用BPU处理器进行模型推理。

算法支持的手势识别类别如下：

```
Background = 0,
ThumbUp = 2,  // 竖起大拇指
Victory = 3,  // “V”手势
Mute = 4,  // “嘘”手势
Palm = 5,  // 手掌
Okay = 11,  // OK手势
ThumbLeft = 12,  // 大拇指向左
ThumbRight = 13,  // 大拇指向右
Awesome = 14  // 666手势
```

示例订阅包含人手关键点信息的ai msg，发布包含手势识别信息的ai msg，用户可以订阅发布的ai msg用于应用开发。

# 编译

## 依赖库


ros package：

- dnn_node
- ai_msgs

dnn_node是在地平线X3开发板上利用BPU处理器进行模型推理的pkg，定义在hobot_dnn中。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select hand_gesture_detection`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hand_gesture_detection \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- mono2d_body_detection package：发布人体、人头、人脸、人手框感知msg
- hand_lmk_detection package：发布人手关键点感知msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式 | 否       | 0/1                  | 0                             |
| model_file_name        | std::string | 推理使用的模型文件                          | 否       | 根据实际模型路径配置 | config/handLMKs.hbm           |
| ai_msg_pub_topic_name  | std::string | 发布包含人手关键点检测结果的AI消息的topic名 | 否       | 根据实际部署环境配置 | /hobot_hand_gesture_detection |
| ai_msg_sub_topic_name_ | std::string | 订阅包含人手框检测结果的AI消息的topic名     | 否       | 根据实际部署环境配置 | /hobot_hand_lmk_detection     |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

运行方式1，使用ros2 run启动：
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

# 启动图片发布pkg
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
ros2 run mono2d_body_detection mono2d_body_detection --ros-args --log-level error &
# 启动人手关键点检测pkg
ros2 run hand_lmk_detection hand_lmk_detection --ros-args --log-level error &
# 启动web展示pkg
ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_hand_gesture_detection --log-level error &


# 启动手势识别pkg
ros2 run hand_gesture_detection hand_gesture_detection

```

运行方式2，使用launch文件启动：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

# 启动launch文件
ros2 launch install/share/hand_gesture_detection/launch/hand_gesture_detection.launch.py

```

### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args --log-level error &
# 启动人手关键点检测pkg
./install/lib/hand_lmk_detection/hand_lmk_detection --ros-args --log-level error &
# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_hand_gesture_detection --log-level error &

# 启动手势识别pkg
./install/lib/hand_gesture_detection/hand_gesture_detection

```

## 注意事项

1. 板端使用launch启动，需要安装依赖，安装命令：`pip3 install lark-parser`。设备上只需要配置一次，断电重启不需要重新配置。

2. 第一次运行web展示需要启动webserver服务，运行方法为:

- cd 到websocket的部署路径下：`cd install/lib/websocket/webservice/`（如果是板端编译（无--merge-install编译选项）执行命令为`cd install/websocket/lib/websocket/webservice`）
- 启动nginx：`chmod +x ./sbin/nginx && ./sbin/nginx -p .`

# 结果分析

## X3结果展示

```
[INFO] [1652147556.777256679] [post process]: track_id: 2, act val: 0, score: 0.999955, threshold: 0.95, max_group_index: 0, max_group_score: 0.999955
[INFO] [1652147556.785721801] [hand gesture det node]: Recved ai msg, frame_id: 1184, stamp: 1652147556_624171980
[INFO] [1652147556.786027119] [preprocess]: target id: 2 has hand roi size: 1
[INFO] [1652147556.786285856] [preprocess]: hand lmk size: 1
[INFO] [1652147556.788131057] [preprocess]: target id: 5 has hand roi size: 1
[INFO] [1652147556.788318256] [preprocess]: hand lmk size: 1
[INFO] [1652147556.792984566] [post process]: track_id: 2, act val: 0, score: 0.999978, threshold: 0.95, max_group_index: 0, max_group_score: 0.999978
[INFO] [1652147556.793831690] [post process]: track_id: 5, act val: 4, score: 0.999978, threshold: 0.5, max_group_index: 4, max_group_score: 0.999978
[WARN] [1652147556.797141860] [hand gesture det node]: publish msg, frame_id: 1184, stamp: 1652147556_624171980
         target id: 2, attr type: gesture, val: 0
         target id: 5, attr type: gesture, val: 4
```

以上log截取了一帧的处理结果，结果显示，订阅到的ai msg中有两个hand（包含人手框和人手关键点检测结果），手势识别算法输出的手势分类结果分别是无手势（分类结果为0）和“嘘”手势（分类结果为4）。

## web效果展示



# 常见问题
