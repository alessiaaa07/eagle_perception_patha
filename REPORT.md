# Report — Cone Detection from LiDAR

## 1. The task and my choice

When I read the task, the first decision was between two paths: **cone detection** (Path A) or **odometry** (Path B). Caterina and I decided which one to do: I chose **Path A: Cone Detection** because I was more interested in taking LiDAR data and turning it into something immediately usable — knowing where the cones are, expressed in the car's frame.

Team Eagle gave us a Dockerfile and a LiDAR bag, and explicitly no package skeleton. At first this felt intimidating, but I soon understood it was the most interesting part of the task: I had to decide the structure, the messages, the node architecture and the pipeline.

## 2. Getting ready: studying before coding

Before writing any code, I spent time researching the tools I would need. I watched videos on YouTube about how Docker works, ROS 2 and how GitHub is structured and the specified commands — including how to write a README — and I read the official documentation of ROS 2, Docker and Git to compare what I was watching with the documentation and build a better background.

This preliminary study was important. Thanks to the videos and documentation, when I later had to create the workspace or fix build errors, I understood what the tools were doing instead of just copying commands without knowing why.

## 3. Setting up the environment

The first practical step was creating the project structure:

```
~/eagle_perception/
├── docker/
├── ws/
└── data/
```

I used Docker from the terminal, with docker run commands, no Docker Desktop or Docker Compose. This kept the setup simple: I only needed Docker Engine, the Dockerfile provided by Eagle, and a few command-line options to mount my workspace and the bag inside the container and forward the display for RViz. I mounted my workspace at /home/eagle/ws and the bag at /home/eagle/data, and used xhost to allow the container to open RViz on my display. The container could be started, stopped and restarted without affecting my system, and the same commands worked every time. 

Inside the workspace I created the two packages required by the task:

- `eagle_perception_interfaces`, for the custom messages;
- `cone_detection`, for the C++ node.

## 4. Defining the messages

Defining the messages was my first real decision. I started with a `Cone.msg` that had position, width, height and colour, because I thought I would need all of them. Then, re-reading the task, I noticed that the minimum requirement was only a position of type `geometry_msgs/Point`. So I simplified the message.

Later in the project, the message evolved again: I added a `type` field for the cone class and a `covariance` field to express the uncertainty of the estimated position. This taught me that message definitions are not written once at the beginning because they evolve with the implementation.

The other two messages were more stable: `ConeArray` (a header plus a list of cones) and `ProcessingTime` (the time taken to process each scan), which the task explicitly required.

## 5. Starting small: the first node

I did not start by writing the full detection pipeline. My first node was minimal: it subscribed to `/lidar/points`, checked that scans were arriving, measured how long it took to process each one, and published that time.

This was a deliberate choice. If I had started directly with ground removal, clustering and transformations, and something had gone wrong, I would not have known whether the problem was in Docker, the bag, the compilation, the ROS communication or my own code. 

The first time I saw the node print `Scan received; processing time: ...` while the bag was playing, it was a small but important moment: it meant the whole chain — container, bag, topic, node, publisher — was working.

Example of the timing logic: 

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto start = std::chrono::steady_clock::now();

  const auto end = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(end - start).count();

  processing_time_pub_->publish(...);
  RCLCPP_INFO(get_logger(), "Scan received; processing time: %.3f ms", elapsed_ms);
}

## 6. Understanding TF and the LiDAR frames

One of the first real conceptual difficulties was understanding the relationship between the lidar frame and the base_link frame. At the beginning, I noticed that the LiDAR point cloud did not coincide with the other point clouds in RViz: the points seemed to be in the wrong place. At first it was not obvious to me why.

I had to realise that the LiDAR is not mounted at the centre of the car and is not perfectly level. The points are expressed in the sensor's frame, but the task requires cone positions in the vehicle frame. The bag provides /tf_static, the static transform between the two frames, and the node uses it to transform every point before doing anything else.

Example:

lidar_to_base = tf_buffer_->lookupTransform(
  "base_link", "lidar", msg->header.stamp, ...);
const Eigen::Vector3d point_base = transform * point_lidar;


Once I added the transform, the point clouds aligned correctly in RViz, and the rest of the pipeline became much clearer. It also explained why the first results looked wrong: without the transform, the cones were simply in the wrong place.

## 7. Removing the ground: from a filter to RANSAC

My first version removed the ground with a simple filter on the z coordinate: keep only the points between two height thresholds. It worked, but it was fragile and the approach did not adapt if the ground was slightly inclined or if the sensor height changed.

I then replaced it with RANSAC, which finds the dominant plane in the point cloud and removes it:

seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
seg.setEpsAngle(0.2617993878);   
seg.setDistanceThreshold(0.10);
seg.setMaxIterations(100);

extract.setIndices(ground_indices);
extract.setNegative(true);       
extract.filter(*cloud_non_ground);

RANSAC finds the ground and removes it. After this change, the results were much cleaner and easier to work with.

## 8. Finding the cones: clustering and filtering

After removing the ground, the remaining points still included barriers, people, buildings and noise. The next step was Euclidean clustering: points that are close to each other are grouped into a single cluster, using a KD-tree to make the search efficient.

But not every cluster is a cone. For each cluster I computed its width, depth and height, and applied a geometric filter: a cone is taller than it is wide, and its dimensions must stay within plausible limits. This filter removes low, wide shapes (barrier pieces, ground noise) and objects that are too large.

For the classification, I distinguished:
- **BIG** cones: height at least 40 cm and width at least 20 cm;
- **SMALL** cones: height between 12 and 40 cm.

| Cluster | h      | w      | d      | Result   | Type  |
| ------- | ------ | ------ | ------ | -------- | ----- |
| 1       | 0.39 m | 0.14 m | 0.15 m | ACCEPTED | SMALL |
| 2       | 0.36 m | 0.13 m | 0.14 m | ACCEPTED | SMALL |
| 3       | 3.86 m | 0.14 m | 0.17 m | REJECTED | —     |
| 4       | 0.79 m | 0.70 m | 0.48 m | REJECTED | —     |
| 5       | 1.90 m | 0.09 m | 0.01 m | REJECTED | —     |

## 9. Estimating the position

The task requires the position of the cone on the ground, not the middle of the points. For this reason, for each valid cluster I took only the points in the lower half and computed the median of their x and y coordinates, setting z to zero.

I chose the median instead of the mean because the mean can be dragged by a few noisy points, while the median is more stable.

I also added a maximum range: cones further than 12 metres are discarded, because at that distance the LiDAR returns fewer points and the estimate is less reliable.

## 10. Expressing uncertainty: the covariance

A cone at 2 metres is estimated more reliably than one at 10 metres. For this reason I added a covariance to each cone, where the uncertainty grows with the distance:


double position_sigma = 0.05 + 0.05 * cone_range;

The covariance is a 3x3 matrix that describes how uncertain the x, y and z position is. In this way, we can treat distant cones with more caution.


covariance[0] = position_sigma * position_sigma;  
covariance[4] = position_sigma * position_sigma;  
covariance[8] = height_sigma * height_sigma;      

## 11. The colour experiment: an honest failure

The task suggested, as a nice-to-have, distinguishing blue from yellow cones using the intensity of the points. Since I had some extra time, I wanted to try to distinguish cones by their colour, understand how to do it, and challenge myself to see if I could do it. First I distinguished blue and yellow cones, then I went further and tried to detect the big orange cones of the start line.

I tried several rules to detect the orange cones. The most promising one combined the cone size, its distance and the intensity of its points:

if (cone_type == "BIG" &&
    cone_range < 6.0 &&
    high_ratio >= 0.25)
{
  cone_color = 3;  // orange
}

This rule matched some start-line cones, but it also matched large blue or yellow cones that happened to be close and reflective. After several attempts, my conclusion was honest: with geometry and intensity alone, I cannot reliably separate orange cones from the other cones or from other objects.

I decided to disable the orange rule and keep only a blue/yellow classification based on the side of the track: blue on the left (y > 0), yellow on the right (y < 0). This was a useful failure.


## 12. Practical problems along the way

Beyond the conceptual difficulties, I hit the usual practical problems:

- A parameter read before being declared, which crashed the node at startup;

- PCL includes not configured in CMake, which broke the build;

- A source file pasted only partially, which produced a confusing expected '}' error;

- A topic that seemed not to exist because the node was not running;

- The confusion between paths inside the container (/home/eagle/ws) and on my own machine (~/eagle_perception);

- A package that seemed missing (Package 'cone_detection' not found) because the workspace had not been rebuilt or sourced correctly.

At some point I also ended up with a duplicated copy of the workspace, which created confusion about which version of the code I was editing and building. We removed the duplicate, thanks also to Angelo, and kept a single workspace mounted inside the container, so that every edit in VS Code was immediately the one being compiled. This taught me to keep one clear source of truth for the code.

Another recurring issue was file ownership: some files inside .git and in the build/ and install/ folders belonged to root, probably because of an earlier sudo command or Docker running as root. This blocked both git commit/git push and colcon build until we fixed the ownership with:

sudo chown -R eagle:eagle /home/eagle/ws

I also had problems with the graphical display: running RViz inside the container required allowing the container to access the local X server. This was solved with:

xhost +local:docker

Without this, RViz could not open a window and the visualisation was not possible.

I also had to debug TF-related problems: sometimes the transform between lidar and base_link was not available at the expected time, and the node had to handle this gracefully instead of crashing.

None of these problems were conceptually hard, but each one taught me the same lesson: before changing code, isolate the problem. Is it the build, the bag, the topic, the container, the permissions, the display, or the logic?

## 13. Visualising and verifying the result

RViz was essential to understand whether the pipeline was working. I set the fixed frame to base_link and added several displays: the raw LiDAR point cloud, the same cloud after the transformation into base_link (the “lidar base” display in RViz), the non-ground cloud, the cone points, and the cone markers.

At the beginning, the raw LiDAR cloud was not aligned with the rest: this was one of the first signs that the transformation between lidar and base_link was needed. Once I added the transform, the cloud aligned correctly with the car.

The other displays let me check each step of the pipeline: the non-ground cloud showed that RANSAC was removing the ground correctly, the cone points showed which clusters had been accepted as cones, and the markers showed the final published positions.

At the beginning I was not sure whether the coloured spheres were my cone markers or just part of the point cloud. By turning the displays on and off, I confirmed that the spheres were the markers I was publishing, placed at the base of each cone cluster — exactly where the cone touches the ground.

Seeing the blue and yellow spheres appear in the right places on top of the point cloud was the moment I felt proud of my project.

Screenshots of the different RViz displays are available here: https://canva.link/lq51l4n08yfvuv7

## 14. Git and documentation

I worked in Git from the beginning, with small commits that each did one thing. When I first tried to push, GitHub asked for a Personal Access Token instead of a password, which was something new for me; I created one and learned how the authentication works.

Sometimes GitHub also asked me to merge branches through a Pull Request on the website: I opened the Pull Request, reviewed the changes, and merged them from there. 

I also learned to prefer targeted commits so that each commit contains exactly what its message says:

Example:

git add README.md
git commit -m "Finalize README"
git push

At one point, some files inside .git belonged to root, which blocked commits and pushes until we fixed the ownership:

sudo chown -R eagle:eagle /home/eagle/ws

The .gitignore keeps build artifacts, logs and the bag out of the repository.


## 15. Results

On the provided bag, the node consistently detects the cones in the scene, typically a small number of cones per scan, with a processing time below the 100 ms budget imposed by the 10 Hz LiDAR.

The cones are published in base_link with their position, class and covariance, and they can be seen in RViz as blue and yellow spheres at the base of the cone clusters, aligned with the point cloud.


## 16. What I would improve

If I had more time, I would like to improve the project in a few ways.

The first would be detecting the orange cones with other tools, since I was not able to do so. This would also make the blue/yellow classification more robust.

I would also improve the cone shape check, to make the cone detection more robust and ensure that only real cones are accepted.

## 17. AI usage

| Tool | What it was used for |
|---|---|
| Perplexity | Explained concepts, project structure, and helped me with the C++ node and parts of the code |
| Perplexity | Helped at the beginning with code in specific files and with debugging |
| Gemini     | Helped with the typical structure of a README and reviewed my C++ code |
| Perplexity | Discussed the detection pipeline and the colour-classification attempt |

