# ROS2 QoS Reliability Non-Functional

tl;dr: using tools such as RViz over WiFi can cause a robot to stall, even when all topics are subscribed using the "best_effort" policy - seemingly it's functionally equivalent to the "reliable" policy...

## Introduction

Mobile robots are very popular in the ROS community,
and they necessitate wireless communication for real-time monitoring and debugging.
Tools such as RViz are commonly used over a WiFi connection to visualize robot sensor data,
path-planning routes, etc.
Unlike fixed robots connected via a wired connection where the bandwidth and latency are fairly constant,
the bandwidth and latency on a WiFi between the mobile robot and a developer's workstation can vary considerably
and often unpredictably as a function of the robot and the developer's relative positions.
This shouldn't be a issue however, as the developer's view of the robot's own perspective on the world is supplemental to the robot's operation - not critical.
This is what the ROS2 topic reliability policies were designed for, a reliable subscription policy that can be used for mission-critical connections,
possibly internal to the robot, and a "best_effort" subscription policy that can be used for non-critical monitoring and debugging.

![ROS2 used over WiFi and Cellular for monitoring](https://github.com/ciandonovan/ros2_qos_reliability/assets/94260580/209ff516-d868-4164-9c77-fbddb73ac407)

The [ROS2 documentation](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html) on QoS polices states that "ROS 2 can be as reliable as TCP or as best-effort as UDP".

As will be demonstrated however, there is seemingly no functional difference between the two policies - both act as a reliable subscriber,
meaning that the publisher will throttle its publishing rate if the subscriber can't keep up,
which is often the case for high-bandwidth topics over a wireless connection.

This means that for mobile robots, where engineers use tools such as RViz monitoring topics over a WiFi link, should that link degrade, the _internal_ communication on the robot also degrades, even when using shared memory!
As such, the behaviour of the robot as a whole under test, and in the field, could be drastically different.

For us this often causes our robot to grind to a halt if we're trying to debug the robot's navigation using RViz and we loose line of sight.
This could be alleviated by only using ROS2 DDS for internal communication on the host, and a ROS bridge to external devices.
But if that's required, then what's the benefit of DDS at all?

Better WiFi coverage and higher bandwidth APs could also alleviated the issue somewhat, but still offers no guarantees, and in principle should not have an effect at all - such non-determinism is a non-starter in many industrial applications.

## Reproduction

The reproducible demo for convenience uses containers to simulate two remote hosts,
but I've also reproduced this issue outside of containers between two real remote hosts,
and that both the FastDDS and Cyclone Iceyorx shared-memory implementations are also affected by slow remote subscribers.

### Prerequisites

Webcam at `/dev/video0` and Podman installed.

### Download and build

```
git clone https://github.com/ciandonovan/ros2_publish_rate.git
cd ros2_qos_reliability
podman build -t $(basename $PWD) .
```

### Run

`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` or `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Simulate a constrained WiFi network on the loopback interface.

`sudo tc qdisc add dev lo root tbf rate 10mbit burst 10mbit latency 50ms`

N.B. use `sudo iftop -i lo` to monitor loopback traffic in real-time.

Run ROS2 camera publisher in one terminal.

`podman run --rm -it --name ros2_publisher --net=host --device=/dev/video0:/dev/video0:rw --env RMW_IMPLEMENTATION ros2_qos_reliability:latest ros2 run v4l2_camera v4l2_camera_node --ros-args --log-level debug`

Run ROS2 camera subscriber in another.

`podman run --rm -it --name ros2_subscriber --net=host --env RMW_IMPLEMENTATION ros2_qos_reliability:latest ros2 topic echo --qos-reliability best_effort /image_raw`

Confirm publisher and subscriber reliability settings

`podman exec -it --env RMW_IMPLEMENTATION ros2_publisher /ros_entrypoint.sh ros2 topic info --verbose /image_raw`

Delete tc qdisc on the loopback when finished.

`sudo tc qdisc del dev lo root`

## Actual behaviour

The "reliable" subscription behaves as expected, the publisher throttles its publishing rate so until the point where all subscribers can receive all published messages given the bandwidth. This can be seen by observing the publishers debug messages.

However, this is also the case for the "best_effort" subscription!

## Comments

It is possible that I've missed something and that some tweak can already be made somewhere to fix this issue.

Even if so, this is not something obvious or documented to my knowledge.

I've spent a long time trying to resolve this issue,
and I can only imagine that many beginners would have just moved on or given up altogether,
thinking that they've done something simple wrong.

ROS2 still has a long way to go to work out-of-the-box as expected.
