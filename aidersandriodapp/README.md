# DJI + Tensorflow Lite for Android

@author: 
Theodosis Georgiou,
Maria Terzi,
Andreas Anastasiou

This app combines the first RosDJI, app which uses DJI sdk and rosclient in order to control various DJI drones through ROS, and the new Tensorflow Lite for android, which gives the ability of lite weight object detection in the app.

# RosDJI

## Android app that connects ROS to DJI drones
The app combines DJI sdk and [rosclient](https://github.com/hibernate2011/RosClient) to receive information from the remote controller and sent them to ROS through rosbridge.

### RosClient: Android app for ROS
Android communicate with ROS(Robot Operating System),based on [rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md)

### Use Libraries
- [EventBus](https://github.com/greenrobot/EventBus)
- [ButterKnife](https://github.com/JakeWharton/butterknife)
- [java_websocket](https://github.com/TooTallNate/Java-WebSocket)
- [ROSBridgeClient](https://github.com/djilk/ROSBridgeClient)
- [AndroidTreeView](https://github.com/bmelnychuk/AndroidTreeView)
- [json-simple](https://github.com/fangyidong/json-simple)


# TensorFlow Lite

TensorFlow Lite is TensorFlow's lightweight solution for mobile and embedded
devices. It enables low-latency inference of on-device machine learning models
with a small binary size and fast performance supporting hardware acceleration.

See the documentation: https://www.tensorflow.org/lite/
Documentation edits can be made here: [tensorflow/lite/g3doc](./g3doc/)


# List of Compatible DJI Aircrafts

1) DJI Spark
2) DJI Mavic 2 (Zoom/Enterprise)
3) DJI M100
4) DJI M210
5) DJI M210-RTK

