

# Devices

```bash
wass@rovi:~$ v4l2-ctl --list-devices
pispbe (platform:1000880000.pisp_be):
        /dev/video20
        /dev/video21
        /dev/video22
        /dev/video23
        /dev/video24
        /dev/video25
        /dev/video26
        /dev/video27
        /dev/video28
        /dev/video29
        /dev/video30
        /dev/video31
        /dev/video32
        /dev/video33
        /dev/video34
        /dev/video35
        /dev/video36
        /dev/video37
        /dev/media1
        /dev/media2

rpivid (platform:rpivid):
        /dev/video19
        /dev/media0

3D USB Camera: 3D USB Camera (usb-xhci-hcd.0-1.1):
        /dev/video0
        /dev/video1
        /dev/media3
```

# Formats

```bash
wass@rovi:~$ v4l2-ctl -d /dev/video0 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 3200x1200
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 2560x720
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 1600x600
                        Interval: Discrete 0.008s (120.000 fps)
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 1280x480
                        Interval: Discrete 0.008s (120.000 fps)
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 640x240
                        Interval: Discrete 0.008s (120.000 fps)
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 3200x1200
                        Interval: Discrete 0.500s (2.000 fps)
                Size: Discrete 2560x720
                        Interval: Discrete 0.500s (2.000 fps)
                Size: Discrete 1600x600
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 1280x480
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 640x240
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
```


# quick test
```bash
sudo apt install ffmpeg
ffplay /dev/video0
```

