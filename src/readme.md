#######################################################################################################################################
# Panthera-HT机械臂 moveit驱动机械臂示例

```
with_gripper_config/                                                          
  ├── launch/                                   # ROS2启动文件目录              
  │   ├── demo.launch.py                        # MoveIt演示启动文件            
  │   ├── hardware.launch.py                    # 硬件接口启动文件              
  │   ├── hardware_moveit.launch.py             # 硬件+MoveIt启动文件           
  │   ├── hardware_moveit_rviz.launch.py        # 硬件+MoveIt+RViz可视化启动文件
  │   ├── move_group.launch.py                  # MoveIt move_group节点启动文件 
  │   ├── moveit_rviz.launch.py                 # MoveIt RViz可视化启动文件     
  │   ├── rsp.launch.py                         # Robot State Publisher启动文件 
  │   ├── setup_assistant.launch.py             # MoveIt Setup Assistant启动文件
  │   ├── spawn_controllers.launch.py           # 控制器加载启动文件            
  │   ├── static_virtual_joint_tfs.launch.py    # 静态虚拟关节TF发布启动文件    
  │   └── warehouse_db.launch.py                # MoveIt数据库启动文件          
  │                                                                             
  ├── config/                                   # 配置文件目录                  
  │   ├── Panthera-HT_description.urdf.xacro    # 机器人URDF描述文件(仿真)      
  │   ├── Panthera-HT_description_hardware.urdf.xacro  #机器人URDF描述文件(硬件)                                                      
  │   ├── Panthera-HT_description.srdf          # MoveIt语义机器人描述文件      
  │   ├── Panthera-HT_description.ros2_control.xacro   # ROS2 Control配置(仿真) 
  │   ├── Panthera-HT_description_hardware.ros2_control.xacro  # ROS2Control配置(硬件)                                                             
  │   ├── joint_limits.yaml                     # 关节限制配置                  
  │   ├── kinematics.yaml                       # 运动学求解器配置              
  │   ├── moveit_controllers.yaml               # MoveIt控制器配置              
  │   ├── moveit.rviz                           # RViz可视化配置文件            
  │   ├── ros2_controllers.yaml                 # ROS2控制器配置(仿真)          
  │   ├── ros2_controllers_hardware.yaml        # ROS2控制器配置(硬件)          
  │   ├── pilz_cartesian_limits.yaml            # Pilz笛卡尔空间限制配置        
  │   └── initial_positions.yaml                # 初始位置配置                  
  │                                                                             
  ├── robot_param/                              # 机器人参数目录                
  │   └── Follower_absolute.yaml                # Follower机械臂绝对位置参数    
  │                                                                                                                                                                 
  ├── CMakeLists.txt                            # CMake构建配置文件             
  ├── package.xml                               # ROS2功能包描述文件            
  └── .setup_assistant                          # MoveIt Setup Assistant配置文件
```

## 环境要求

- ubuntu24.04
- ros的jazzy版本

### 安装依赖
  一次性安装所有依赖：
  sudo apt install \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller 

## 示例代码说明

### 1. moveit驱动真实机械臂 (hardware_moveit_rviz.launch.py)
**功能**: 实现moveit中轨迹规划的路径，通过controller实现轨迹插补，以驱动真实的机械臂。

**特点**:
- moveit中机械臂规划和执行的结果发送到到现实中，实现rviz和现实一样的效果。

**运行方法**:
1:检查串口是否连上 ls dev/ttyACM*
2：去到相应的工作空间 cd to_ws
3: source install/setup.base
4：  ros2 launch with_gripper_config hardware_moveit_rviz.launch.py

### 2. moveit只是在rviz中路径规划 (demo.launch.py)
**功能**: 实现moveit中轨迹规划的路径，通过controller实现轨迹插补，以驱动真实的机械臂。

**特点**:
- moveit中机械臂规划和执行的结果发送到到现实中，实现rviz和现实一样的效果。

**运行方法**:
1:检查串口是否连上 ls dev/ttyACM*
2：去到相应的工作空间 cd to_ws
3: source install/setup.base
4：  ros2 launch with_gripper_config demo.launch.py

#######################################################################################################################################
# Panthera-HT机械臂 moveit驱动gazebo中的机械臂示例
```
  gripper_gazebo/                                                               
  ├── launch/                                   # ROS2启动文件目录              
  │   ├── gazebo.launch.py                      # Gazebo仿真启动文件（核心）    
  │   ├── gazebo.launch.xml                     # Gazebo启动文件（XML格式）     
  │   ├── gazebo_moveit.launch.py               # Gazebo+MoveIt集成启动文件     
  │   └── gazebo_moveit.launch.xml              #                               
  Gazebo+MoveIt启动文件（XML格式）                                              
  │                                                                             
  ├── config/                                   # 配置文件目录                  
  │   ├── gazebo_ros2_control.xacro             # Gazebo专用ROS2 Control配置    
  │   └── ros2_controllers.yaml                 # Gazebo控制器参数配置          
  │                                                                             
  ├── urdf/                                     # URDF模型文件目录              
  │   ├── Panthera-HT_gazebo.urdf.xacro         # Gazebo专用机器人URDF描述文件  
  │   └── gazebo.xacro                          # Gazebo插件和属性配置          
  │                                                                             
  ├── worlds/                                   # Gazebo世界文件目录            
  │   └── empty.world                           # 空白仿真世界文件              
  │                                                                             
  ├── CMakeLists.txt                            # CMake构建配置文件             
  └── package.xml                               # ROS2功能包描述文件
```
## 环境要求

- ubuntu24.04
- ros的jazzy版本

### 安装依赖
  一次性安装所有依赖： 
  sudo apt install \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller 

## 示例代码说明
### 1. moveit驱动gazebo中仿真机械臂 (demo.launch.py)
**功能**: 实现moveit中的轨迹规划的执行结果驱动gazebo仿真中的机械臂

**特点**:
- moveit中机械臂规划和执行的结果发送到gazebo仿真中的机械臂中，实现驱动moveit中机械臂的效果。

**运行方法**:
1:检查串口是否连上 ls dev/ttyACM*
2：去到相应的工作空间 cd to_ws
3: source install/setup.base
4：  ros2 launch gripper_gazebo gazebo_moveit.launch.py

#######################################################################################################################################
# Panthera-HT机械臂 几个驱动机械臂例程

```
  srcsrc/                                                            
  ├── examples/                                 # 示例程序目录                  
  │   ├── 0_robot_get_state.cpp                 # 获取机器人状态示例            
  │   ├── 0_robot_set_zero.cpp                  # 机器人归零示例                
  │   ├── 1_PD_control.cpp                      # PD控制示例                    
  │   ├── 1_PosVel_control.cpp                  # 位置速度控制示例              
  │   ├── 2_joint_impedance_control.cpp         # 关节阻抗控制示例              
  │   ├── 3_cartesian_impedance_control.cpp     # 笛卡尔阻抗控制示例            
  │   ├── cartesian_impedance_ab_motion.cpp     # 笛卡尔阻抗AB运动示例          
  │   └──pure_cartesian_impedance_control.cpp  # 纯笛卡尔阻抗控制示例          
  │                                                                                           
  ├── src/                                      # 源代码目录                    
  │   ├── crc/                                  # CRC校验相关源码               
  │   ├── hardware/                             # 硬件接口相关源码              
  │   ├── panthera/                             # Panthera机器人核心源码        
  │   ├── parse_robot_params.cpp                # 机器人参数解析                
  │   └── serial_driver.cpp                     # 串口驱动程序                  
  │                                                                             
  ├── include/                                  # 头文件目录                    
  │   ├── crc/                                  # CRC校验头文件                 
  │   ├── hardware/                             # 硬件接口头文件                
  │   └── panthera/                             # Panthera机器人头文件          
  │                                                                             
  ├── Panthera-HT_description/                  # 机器人描述包                  
  │   ├── config/                               # 配置文件                      
  │   ├── launch/                               # 启动文件                      
  │   ├── meshes/                               # 3D网格模型文件（STL）         
  │   ├── textures/                             # 纹理文件                      
  │   ├── urdf/                                 # URDF机器人描述文件            
  │   ├── CMakeLists.txt                        # CMake构建配置                 
  │   ├── package.xml                           # ROS2功能包描述                
  │   └── export.log                            # 导出日志                      
  │                                                                             
  ├── robot_param/                              # 机器人参数目录                
  │   └── motor_param/                          # 电机参数配置                  
  │                                                                             
  ├── msg/                                      # ROS2消息定义目录              
  │   └── motor_msg/                            # 电机消息定义                  
  │                                                                             
  ├── third_part/                               # 第三方库目录                  
  │   ├── lcm/                                  # LCM通信库                     
  │   └── serial_cmake/                         # 串口通信CMake配置             
  │                                                                             
  └── urdf/                                     # URDF文件目录                  
```
### 安装依赖
  sudo apt install \
    libyaml-cpp-dev \
    libserialport-dev \
    libeigen3-dev \
    libboost-all-dev \
    liburdfdom-dev \
    ros-jazzy-pinocchio \
    ros-jazzy-ament-cmake 
    ## 示例代码说明
### 1. 运行笛卡尔空间的阻抗控制 (pure_cartesian_impedance_control.cpp)
**功能**: 实现笛卡尔空间的阻抗控制

**特点**:
- 运行代码3秒后，机械臂将记录当前的位置作为固定位置。后机械臂固定到该固定位置，对机械臂施加扰动后机械臂体现阻抗特性。

**运行方法**:
1:检查串口是否连上:ls dev/ttyACM*
2：进入以下路径/ros_ws/src/pure_src/build 
3:将build里面原来编译好的文件全部删掉:rm -rf *
4:在build路径下运行:cmake ..
5：在build路径下编译:make
6:在build路径下运行：./pure_cartesian_impedance_control

### 2. 运行笛卡尔空降阻抗控制（但是给定每个关节初始值让回到原来位置） (3_cartesian_impedance_control.cpp)
**功能**: 实现笛卡尔空间的阻抗控制，但是每一个轴都有回到原来初始位置的趋势

**特点**:
- 机械臂将直接前往到写死的那个位置，机械臂固定到该固定位置，对机械臂施加扰动后机械臂体现阻抗特性。但是对机械臂侧边进行扰动，机械臂将回到原来的位置。

**运行方法**:
1:检查串口是否连上:ls dev/ttyACM*
2：进入以下路径/ros_ws/src/pure_src/build 
3:将build里面原来编译好的文件全部删掉:rm -rf *
4:在build路径下运行:cmake ..
5：在build路径下编译:make
6:在build路径下运行：./3_cartesian_impedance_control

### 3. 实现在A和B两点间的来回运动的阻抗控制 (cartesian_impedance_ab_motion.cpp)
**功能**: 机械臂在A和B点间来回运动，实现其过程中的阻抗控制

**特点**:
- 启动程序后3秒，机械臂将记录当前位置A点，后终端要求输入相对于A的另外个位置B（一般设为往x方向偏移0.3，y和z方向偏移为0）

**运行方法**:
1:检查串口是否连上:ls dev/ttyACM*
2：进入以下路径/ros_ws/src/pure_src/build 
3:将build里面原来编译好的文件全部删掉:rm -rf *
4:在build路径下运行:cmake ..
5：在build路径下编译:make
6:在build路径下运行：./cartesian_impedance_ab_motion

### 4. 运行关节空间的阻抗控制 (2_joint_impedance_control.cpp)
**功能**: 实现关节空间的阻抗控制

**特点**:
- 机械臂将直接前往到写死的那个位置，机械臂固定到该固定位置，机械臂会回到原来位置

**运行方法**:
1:检查串口是否连上:ls dev/ttyACM*
2：进入以下路径/ros_ws/src/pure_src/build 
3:将build里面原来编译好的文件全部删掉:rm -rf *
4:在build路径下运行:cmake ..
5：在build路径下编译:make
6:在build路径下运行：./2_joint_impedance_control
