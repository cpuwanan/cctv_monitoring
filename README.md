# cctv_monitoring

Tag:
- rospy
- rosservice

### Demo 1: Simple usage

Before using, give permission to script files
```
$ sudo chmod +x src/ms_perception/scripts/fake_image_reader.py
$ sudo chmod +x src/ms_perception/scripts/fake_object_detector.py
$ sudo chmod +x src/ms_perception/scripts/fake_policy_manager.py
```

Run
```
$ roslaunch ms_perception demo_findobjects_service.launch
```

Snapshot of results

![img1](pictures/bay_status.png)

Rostopic of `/all_bays`

![img2](pictures/rostopic_all_bays.png)
