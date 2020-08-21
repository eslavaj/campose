# campose
This is an application example to recover camera pose from monocular camera images.
This application uses OpenCV.
The HW platform is Jetson TX2. Currently, most of the code is executed on CPU. I am planning to move more a more code to GPUs as I progress on this project.
I used Kitty dataset sequence 3 to test it, please know this is a Work in progress project, the following points needs to be addressed nextly:
- Global/bundle adjustment
- Fusion with GPS/IMU
- Optimization
- Port to GPU (whenever it is useful and possible)
