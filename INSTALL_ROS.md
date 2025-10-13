# ROS安装指南

## 安装ROS 2 Humble（Ubuntu 24.04）

### 1. 设置软件源
```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. 更新软件包列表
```bash
sudo apt update
```

### 3. 安装ROS 2
```bash
sudo apt install -y ros-humble-desktop
```

### 4. 安装开发工具
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete
```

### 5. 初始化rosdep
```bash
sudo rosdep init
rosdep update
```

### 6. 设置环境
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. 安装额外依赖
```bash
sudo apt install -y python3-pip build-essential
pip3 install -U colcon-common-extensions
```

## 安装完成后

### 编译项目
```bash
cd /home/sss/mapping
colcon build
source install/setup.bash
```

### 运行bag包处理
```bash
./scripts/process_bag.sh ./data.bag ndt
```

## 注意事项
- ROS 2使用colcon而不是catkin_make
- 环境设置文件在install/setup.bash
- 某些ROS 1的bag包可能需要转换工具
