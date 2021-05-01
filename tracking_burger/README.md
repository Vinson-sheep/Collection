# Burger Tracking Control Unit

By this class, turtlebot3 can trace the path smoothly without any consideration of controling. Initualization is expected to be executed before using. Don't worry. Operation is extreme simple and you can grasp by a minute.

[reference](https://raw.githack.com/Vinson-sheep/ros_package/master/tracking_burger/docs/html/index.html)

## demo

``````

Global_Planning::TrackUnit::Ptr tu_ptr(new Global_Planning::TrackUnit(nh, "cmd_vel"));
tu_ptr->set_frame_id_burger("base_footprint");
tu_ptr->set_frame_id_odom("odom");
tu_ptr->set_fw_lk_distance(0.5);

ros::Rate r(1);
while(ros::ok()){
    tu_ptr->pub_control(path);
    ros::spinOnce();
    r.sleep();
}
``````


