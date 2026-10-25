# E-Agle TRT AS — Perception Software Task (Path A: Cone Detection)

ROS 2 Humble C++ node that detects cones from a LiDAR recording and publishes their positions in the vehicle frame (`base_link`).

## Pipeline

1. Subscribe to `/lidar/points` (frame `lidar`);
2. Transform the scan into `base_link` using the static transform provided by the bag;
3. Remove the ground plane with RANSAC;
4. Cluster the remaining points (Euclidean clustering);
5. Filter clusters by cone-like proportions (height, width, depth, range);
6. Estimate each cone's ground position and classify it (small / big);
7. Publish cones, RViz markers, and per-scan processing time.

## Repository structure

```text
eagle_perception/
├── docker/
│   └── Dockerfile
├── src/
│   ├── cone_detection/
│   │   ├── config/
│   │   │   └── cone_detection.yaml
│   │   ├── launch/
│   │   │   └── cone_detection.launch.py
│   │   ├── src/
│   │   │   └── cone_detection_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── eagle_perception_interfaces/
│       ├── msg/
│       │   ├── Cone.msg
│       │   ├── ConeArray.msg
│       │   └── ProcessingTime.msg
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── LICENSE
├── .gitignore
├── README.md
└── REPORT.md
```

## Requirements

- Docker;
- A running X server (Linux desktop) to display RViz.

## 1. Get the LiDAR recording

Download the bag from the link provided in the task description, unzip it, and place it at:

```text
data/perception_bag/
```

The bag is not included in this repository.

## 2. Build the Docker image

From the repository root:

```bash
docker build -t eagle-perception:humble -f docker/Dockerfile .
```

## 3. Create and enter the container

```bash
xhost +local:docker
docker run -it \
  --name eagle_perception \
  --user eagle \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ~/eagle_perception:/home/eagle/ws \
  --volume ~/eagle_perception/data:/home/eagle/data \
  --network host \
  eagle-perception:humble \
  bash
```

To re-enter the container in later sessions, first:

```bash
docker start eagle_perception
```

Then:

```bash
docker exec -it eagle_perception bash
```

## 4. Build the workspace

Inside the container:

```bash
cd ~/ws
colcon build
source install/setup.bash
```

## 5. Run the perception node

Open a separate terminal, enter the container and play the bag:

```bash
docker exec -it eagle_perception bash
ros2 bag play data/perception_bag --loop
```

In another terminal:

```bash
docker exec -it eagle_perception bash
cd ~/ws
source install/setup.bash
ros2 launch cone_detection cone_detection.launch.py
```

## 6. Visualize in RViz

In another terminal:

```bash
docker exec -it eagle_perception bash
rviz2
```

In RViz set **Fixed Frame** to `base_link`, then add:

- `/lidar/points` (PointCloud2) - raw scan from LiDAR;
- `/perception/lidar_base` (PointCloud2) - scan transformed into the vehicle frame;
- `/perception/non_ground` (PointCloud2) - points after ground removal;
- `/perception/cone_points` (PointCloud2) - points belonging to detected cones;
- `/perception/cones_markers` (MarkerArray) - detected cones.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Input: raw LiDAR scan, frame `lidar` |
| `/perception/cones` | `eagle_perception_interfaces/msg/ConeArray` | Detected cones, frame `base_link` |
| `/perception/cones_markers` | `visualization_msgs/msg/MarkerArray` | Cone markers for RViz |
| `/perception/processing_time` | `eagle_perception_interfaces/msg/ProcessingTime` | Processing time per scan (ms) |
| `/perception/lidar_base` | `sensor_msgs/msg/PointCloud2` | Scan transformed into `base_link` |
| `/perception/non_ground` | `sensor_msgs/msg/PointCloud2` | Points after ground removal |
| `/perception/cone_points` | `sensor_msgs/msg/PointCloud2` | Points belonging to detected cones |

## Parameters

All thresholds are loaded from `src/cone_detection/config/cone_detection.yaml`.

Parameters can also be changed at runtime without restarting the node, for example:

```bash
ros2 param set /cone_detection_node max_cone_range 6.0
```

## Results

On the provided bag, the node typically detects few cones per scan, with a processing time of about 60-90 ms per frame.