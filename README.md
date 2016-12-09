# virtualFence

By Jaehyung Jang.<br>
This is the term project space of the Robot Software Platform class.<br><br>


### Purpose<br>

An algorithm package for implementing a virtual fence in ROS.<br>
This algorithm requires a robot package and launch package. However, we provide packages and so on to run the simulation.

### Description<br>

- Rotate 360 degrees from each corner of the fence and take a picture.
- Compares the current image with the stored image to determine the current position.
- Moves out of the fence following the specified corner.

### Package<br>

- Robot Package<br>
It has a plug-in that can move the robot, and reads RGB images.
- Algorithm Package<br>
This prepares the algorithm by receiving the image from the robot package.<br>
Then, when the current image comes in, it executes the algorithm and returns the direction of the robot.
- Launch Package<br>
It calls the gazebo simulation world and executes the robot package and the algorithm package.
