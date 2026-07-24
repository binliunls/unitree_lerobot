# Packaging gscam2 For Other Hosts

Use this path when another Jetson Thor / ROS 2 Jazzy host should install the
same source-built `gscam2` without rebuilding it manually in a workspace.

The package built here installs into `/opt/ros/jazzy` and is named
`ros-jazzy-gscam2`. After installation, `source /opt/ros/jazzy/setup.bash`
should expose `gscam2` like a normal ROS package.

## Build The Debian Package

On the build host:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
GSCAM2_SRC=/home/unitree/dev/gscam2 ./scripts/package_gscam2_deb.sh
```

The output is written to `dist/`, for example:

```text
dist/ros-jazzy-gscam2_0.0.2-1_arm64.deb
```

## Install On Another Host

Copy this repository and the `.deb` to a matching Ubuntu 24.04 arm64 / ROS 2
Jazzy host. The Debian package provides the `gscam2` ROS package under
`/opt/ros/jazzy`; the repository provides the camera launch scripts,
calibration files, CycloneDDS config, and validation tools.

From the build host:

```bash
rsync -a /home/unitree/dev/sensing_wrist_gstreamer/ unitree@TARGET:/home/unitree/dev/sensing_wrist_gstreamer/
scp /home/unitree/dev/sensing_wrist_gstreamer/dist/ros-jazzy-gscam2_0.0.2-1_arm64.deb unitree@TARGET:/home/unitree/dev/sensing_wrist_gstreamer/
```

On the target host:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
sudo apt install ./ros-jazzy-gscam2_0.0.2-1_arm64.deb
source /opt/ros/jazzy/setup.bash
ros2 pkg executables gscam2
```

Expected output includes:

```text
gscam2 gscam_main
```

## Optional rosdep Override

If another ROS package declares:

```xml
<depend>gscam2</depend>
```

add the local rosdep rule on that host:

```bash
sudo cp /home/unitree/dev/sensing_wrist_gstreamer/rosdep/gscam2-local.yaml /etc/ros/rosdep/sources.list.d/gscam2-local.yaml
sudo sh -c 'echo "yaml file:///etc/ros/rosdep/sources.list.d/gscam2-local.yaml" > /etc/ros/rosdep/sources.list.d/50-gscam2-local.list'
rosdep update
```

Then `rosdep install` can resolve the `gscam2` key to the Debian package
`ros-jazzy-gscam2`.

## Notes

- This is a local Debian package, not an official ROS buildfarm release.
- Build and install on the same OS/architecture family: Ubuntu 24.04 arm64,
  ROS 2 Jazzy.
- NVIDIA Jetson GStreamer plugins such as `nvvidconv` still come from JetPack
  on the target host; they are not bundled into this package.
- The package depends on ROS and GStreamer runtime libraries, but not on the
  camera-specific launch scripts in this repo.
