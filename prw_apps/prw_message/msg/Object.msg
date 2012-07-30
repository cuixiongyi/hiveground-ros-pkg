# A pose, reference frame, timestamp, and type
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
sensor_msgs/PointCloud2 cloud
string name
int32 type

# object definitions
int32 UNKNOW = 0
int32 CUBE = 1
int32 SPHERE = 1
