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

- **Robot Package**(essential)<br>
It has a plug-in that can move the robot, and reads RGB images.<br>
**You should prepare this package.** We used the turtlebot package.
- **Algorithm Package**(we provided)<br>
This prepares the algorithm by receiving the image from the robot package.<br>
Then, when the current image comes in, it executes the algorithm and returns the direction of the robot.
- **Launch Package**(optional)<br>
It calls the gazebo simulation world and executes the robot package and the algorithm package.

### How to Use<br>
- Check the topic of the image coming from the robot package.
- Modify the name of the image topic in the algorithm package. (Example: camera/rgb/raw_image)
- Check the topic of robot status coming from the robot package.
- Modify the name of the service in the algorithm package. (Example: gazebo/get_model_state)
- Then you are ready to use.
- Moving the robot to the node, such as teleop, press the 'space' button in four hit the fence.
- Finally, ressing the n button activates the algorithm.

### Future Plans
- The node receiving commands from the keyboard is unstable. We want to improve this.
- Now we use line features in images. We will add point features in the future.
![alt tag](http://dl.dropbox.com/s/00og2q900ck9l93/system_environment.jpg)
![s ss](http://dl.dropbox.com/s/435p4w6dpo418wm/block_diagram.jpg)
