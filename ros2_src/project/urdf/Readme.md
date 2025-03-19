https://automaticaddison.com/how-to-convert-a-xacro-file-to-urdf-and-then-to-sdf/

```bash
xacro turtlebot3_burger_camera.xacro > turtlebot3_burger_camera.urdf
gz sdf -p turtlebot3_burger_camera.urdf > ../models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf
sed -i 's|model://project/models/turltebot3_burger_camera/|model://turltebot3_burger_camera/|g' ../models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf
cd ~/ros2_ws && colcon build --packages-select project
```
