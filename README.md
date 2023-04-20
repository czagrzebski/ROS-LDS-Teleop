# cs431-lds-teleop
Laser Distance Sensor Display (LDS) with pre-collision alerts for ROS Mobile Robots that use `/scan` and `/cmd_vel` topics. Built using pygame and rospy. 

## How it works
- Retrieves 360 degree laser scan data from the `/scan` topic
- Checks for potential collision by checking to see if measured distances are below specific collision threshold
- Alert user if possible collision is detected
- Render the LiDAR scan data relative to the robot model using matrix transformations
- Check for keypresses to command robot movement
- Publish desired linear and angular velocity to `/cmd_vel` topic
