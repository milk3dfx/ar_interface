AR Interface ROS Node
========
The node augment image received from robot with labels of recognized objects, and publish new image.

To run the node use:
```
$ rosrun ar_interface ar_interface
```

Listening topics:
* /camera/rgb/image_raw
* /web_mouse
* /object_recognition_listener/recognized_objects

Publishing topics:
* /augmented_image_raw

Calling services:
* /object_recognition_listener/recognized_objects