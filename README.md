# ps4_stereo_camera
PS4 stereo camera driver using the V4L2 API. Unlike the [ps4eye](https://github.com/longjie/ps4eye) package which is using gstreamer/gscam, it is using the V4L2 API directly and has lower CPU consumption.

# Credits

Code in this package is based partly on two other projects:

* The V4L2/Minoru packages by Bob Mottram and Giacomo Spigler (See [libv4l2cam](https://github.com/bashrc/libv4l2cam) and [MinoruWebcam](http://code.google.com/p/sentience/wiki/MinoruWebcam))
* The [ps4eye](https://github.com/longjie/ps4eye) ROS package. 


## WARNING

* This package is experimental and not completed.
* You need to hack (cut and solder) the PS4 camera's cable. It may damage your device. Do it at your own risk.
* The PS4 camera is USB 3.0 only and is not compatible with USB 2.0 systems.

## Requirements

* You need a ps4eye camera [PlayStation Camera CUH-ZEY1J](http://www.jp.playstation.com/ps4/peripheral/cuhzey1j.html)

* You need to hack the cable. See [PS4eye](http://ps4eye.tumblr.com/post/79572946666/more-photos-of-cable-wiring-to-clarify-how-the).

* You may need newer linux kernel. Starting from Kernel version 3.17.3 things seem to work.

  If you are using ubuntu, you might want to check [this page](http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.17.3-vivid/) for updating your kernel.
* You'll need Pyusb 1.0 or the script executed in the udev rules will silently fail.
    sudo pip install --pre pyusb

## Usage

0. Run create_udev_rules. Enter sudo password to place /etc/udev/rules.d/91-ps4eye.rules. You need this step only once.
```
$ rosrun ps4_stereo_camera create_udev_rules
```

1. Plug your ps4eye into a free USB 3.0 port.

2. Check if you can obtain the  image from ps4 camera by webcam software (cheese, camorama, etc).

3. Run stereo.launch.
```
$ roslaunch ps4_stereo_camera ps4_stereo_camera_default.launch
```
