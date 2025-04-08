# Human Detection and Autonomous Victim Search

Данный репозиторий содержит инструкции по запуску двух режимов на базе ROS Noetic: **Human Detection** и **Autonomous Victim Search** как в симуляции, так и на реальном роботе. Ниже описаны необходимые настройки и команды для запуска режимов. После представлена секция по визуализации. В конце расположены команды включения/выключения представленных режимов через ROS-сервисы.

Если речь идет о работе с реальным роботом, то предполагается, что визуализация и включение/выключение режимов будут всегда выполняться на компьютере пользователя.

---

## Dependencies

- clone repo from `https://gitlab.com/lirs-kfu/engineer-gazebo-model.git` to your workspace and build it

## Build

- clone this repo to your workspace and build it
- `source devel/setup.bash`

## Run

### Sequence #1: for simulation:

- spawn turtlebot3 robot: `roslaunch sar turtlebot3_gazebo.launch`
- `roslaunch hum_det stereo_img_proc.launch`

### Sequence #2: for real:

- stereo camera of engineer robot: `roslaunch hum_det engineer_stereo.launch`

### Sequence #3: Human Detection:

*pay attention: you should set value of the constant `IS_LAPTOP` in the file `hum_det/src/yolov10_node.h`: **for simulation**: `true`; **for real**: `false`*

*pay attention: you should set correct model paths in the function `get_model_path` in the file `hum_det/src/yolov10_node.h`*

- `rosrun hum_det yolov10_node`

### Sequence #4: Autonomous Victim Search:

*pay attention: you should set value of the arg `is_turtlebot3` in the file `search_rescue.launch`: **for simulation**: `true`; **for real**: `false`*

- **for simulation**: ROS Navigation Stack: `roslaunch sar turtlebot3_move_base.launch`
- **for real**: ROS Navigation Stack: `roslaunch sar engineer_move_base.launch`
- `roslaunch sar search_rescue.launch`
- `roslaunch hum_det avs_main.launch`

## Visualization

### Sequence #1: ROS Navigation Stack:

- **for simulation**: `roslaunch sar turtlebot3_rviz.launch`
- **for real**: `roslaunch sar engineer_rviz.launch`

### Sequence #2: отобразите данный список ROS-топиков удобным для Вас способом:

1. `/yolov10_node/stereo/left/det_image`
2. `/depth_map`
3. `/depth_map_roi`

## On and Off

### Sequence #1: Human Detection:

- turn on: `rosservice call /det_mode_switch "is_on: true"`
- turn off: `rosservice call /det_mode_switch "is_on: false"`

### Sequence #2: Autonomous Victim Search:

- turn on: `rosservice call /det_group_mode_switch "is_on: true"`
- turn off: `rosservice call /det_group_mode_switch "is_on: false"`
