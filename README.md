# ROS2 QoS Reliability Non-Functional

## Prerequisites

Laptop with webcam at `/dev/video0` and Podman installed.

## Download and build

```
git clone https://github.com/ciandonovan/ros2_publish_rate.git
cd ros2_qos_reliability
podman build -t $(basename $PWD) .
```

## Reproduction

`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` or `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Simulate a constained WiFi network on the loopback interface.

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

## Analysis

The reliable subscription behaves as expected, the publisher throttles its publishing rate so until the point where all subscribers can receive all published messages given the bandwidth.

However, this is also the case for the "Best Effort" subscription!

This means that for mobile robots, where engineers use tools such as RViz monitoring topics over a WiFi link, should that link degrade, the _internal_ communication on the robot also degrades, even when using shared memory!
As such, the behaviour of the robot as a whole under test, and in the field, could be drasticaly different.
There is no documentation either to elivate this issue, or even warn about it.
