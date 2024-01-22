# ROS node for trisonica mini

This node will publish to ROS all of the measurements provided by the trisonica mini over its serial connection. Run using `rosrun ./trisonica_ros trisonica.py`. 

## Setting parameters

You can set the parameters on the trisonica through minicom using the following types of commands. See calibration_guide.md for more detail. 
1. Run minicom: `minicom -s`
2. Change hardware flow control to off (`F`)
3. Set serial device (`A`)
4. Hit `ctrl-c`, this opens a command line interface with a prompt: `>`
5. Run desired command, e.g.: `> outputrate 40`
6. Save parameters: `> nvwrite`