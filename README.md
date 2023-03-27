# Embodied_AI_Platform

# Requirement
1. Ubunut 20.04 (Shared Machine will be availble in Games Lab)
2. ROS Noetic

# Setup
## 1. Workspace Preparation
create a seprate catkin_workspace to avoid any conflicts with your current ones

```console
mkdir -p ~/catkin_ws_emb/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```


## 2. Download Repo
1. remove the src created in the previous step
```console
rm -r src
```
2. download this repo
```console
git clone https://github.com/ai4ce/Embodied_AI_Platform.git

```
3. catkin_make


## 3. Run Demo
```console
roslaunch emb_dev dev.launch
```
you should see console output like this 

```console
Published maze: 10 x 20
Publish as occupancy grid ...
```
