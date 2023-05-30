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
This shouldn't be a issue however, as a momentary drop in signal should only impact the developer's live data display, and not impede the robot's task.
This is what the ROS2 topic reliability policies were designed for, a reliable subscription policy that can be used for mission-critical connections,
possibly internal to the robot, and a "best_effort" subscription policy for non-critical telemetry and other monitoring.

As will be demonstrated however, there is seemingly no functional difference between the two policies - both act as a reliable subscriber,
meaning that the publisher will throttle its publishing rate if the subscriber can't keep up,
which is often the case for high-bandwidth topics over a wireless connection.

For us this often causes our robot to grind to a halt if we're trying to debug the robot using RViz and we loose line of sight.
Loss of data on the developer side is unavoidable in this scenario,
but it absolutely should not be the case that the robot is in any way impacted by there external factors.

The reproducible demo uses containers to simulate two remote hosts for convenience,
but I've also reproduced this issue outside of containers between two remote hosts,
and that both the FastDDS and Cyclone Iceyorx shared-memory implementations are also affected by slow remote subscribers.

This means that for mobile robots, where engineers use tools such as RViz monitoring topics over a WiFi link, should that link degrade, the _internal_ communication on the robot also degrades, even when using shared memory!
As such, the behaviour of the robot as a whole under test, and in the field, could be drastically different.
There is no documentation either to elevate this issue, or even warn about it.

## Reproduction

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

```
podman run --rm -it --name ros2_publisher --net=host --device=/dev/video0:/dev/video0:rw --env RMW_IMPLEMENTATION ros2_qos_reliability:latest ros2 run v4l2_camera v4l2_camera_node --ros-args --log-level debug
```

```
podman run --rm -it --name ros2_subscriber --net=host --env RMW_IMPLEMENTATION ros2_qos_reliability:latest ros2 topic echo --qos-reliability best_effort /image_raw
```

Confirm publisher and subscriber reliability settings

```
podman exec -it --env RMW_IMPLEMENTATION ros2_publisher /ros_entrypoint.sh ros2 topic info --verbose /image_raw
```

Delete tc qdisc on the loopback when finished.

`sudo tc qdisc del dev lo root`

## Expected behaviour

## Actual behaviour

The "reliable" subscription behaves as expected, the publisher throttles its publishing rate so until the point where all subscribers can receive all published messages given the bandwidth.

However, this is also the case for the "best_effort" subscription!

## Comments

It is possible that I've missed something and that some tweak can already be made somewhere to fix this issue.

Even if so however, this is not something obvious or documented to my knowledge.

I've spent many hours trying to resolve this issue,
and I can only imagine that many other beginners would have just moved on to a different project or given up altogether.

ROS2 still has a long way to go to work out-of-the-box as expected.
