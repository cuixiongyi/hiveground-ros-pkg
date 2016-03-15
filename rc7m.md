# RC7M Controller #

[RC7M](http://www.denso-wave.com/en/robot/product/controller/rc7m__Spec.html)


**WARNING** only tested with RC7M-5/6CAA model

# Details #
This package is a plug-in of [hg\_cpp](hgcpp.md) package.


## Launch File (.launch) ##
An example of launch code.
```
  <!-- RC7M -->
  <node name="vp6242" pkg="hg_cpp" type="hg_ros" output="screen">
    <param name="simulate" value="true"/>
    <param name="loop_rate" value="50.0"/>
    <param name="joint_publish_rate" value="10.0"/>
    <param name="diagnostic_publish_rate" value="10.0"/>      
    <rosparam file="$(find rc7m)/parameters/vp6242.yaml" command="load" />    
  </node>
```

## Parameter File (.yaml) ##
An example for parameter file for VP-6242G robots.
```
#All joint information
#0.1 rad/s -> 5.729577951 deg/s
joints: {
  J1: {
    #-160 / 160
    type: rc7m_joint,
    upper_limit: 2.792526804,
    lower_limit: -2.792526804,
    position_offset: 0.0, #for ROS joint state
    velocity_limit: 3.0
  },
  J2: {
    #RC7M software limit -120 / 120
    type: rc7m_joint,
    upper_limit: 2.094395103,
    lower_limit: -2.094395103,
    position_offset: 0.0,  #for ROS joint state
    velocity_limit: 3.0  
  },
  J3: {
    #RC7M software limit  19 / 160
    type: rc7m_joint,
    #upper_limit: 2.792526804,
    #lower_limit: 0.331612558,    
    upper_limit: 1.221730477,
    lower_limit: -1.239183769,    
    position_offset: -1.570796327, #for ROS joint state 
    velocity_limit: 3.0
  },
  J4: {
    #RC7M software limit -160 / 160
    type: rc7m_joint,
    upper_limit: 2.792526804,
    lower_limit: -2.792526804,    
    position_offset: 0.0,  #for ROS joint state
    velocity_limit: 3.0
  },
  J5: {
    #RC7M software limit -120 / 120
    type: rc7m_joint,
    upper_limit: 2.094395103,
    lower_limit: -2.094395103,
    position_offset: 0.0,  #for ROS joint state
    velocity_limit: 3.0
  },
  J6: {
    #RC7M software limit -360 / 360
    type: rc7m_joint,
    upper_limit: 6.283185308,
    lower_limit: -6.283185308,
    position_offset: 0.0,  #for ROS joint state
    velocity_limit: 3.0
  }
}

#All controller information
controllers: {
  rc7m_arm1 : {
    type: rc7m_controller,
    name: arm1,
    rate: 125,
    ip: 10.0.0.101,
    port: 5007,    
    joints: [J1, J2, J3, J4, J5, J6]
  }
}
```