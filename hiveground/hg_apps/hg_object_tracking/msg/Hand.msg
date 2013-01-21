geometry_msgs/Vector3 arm_centroid
geometry_msgs/Vector3 arm_eigen_value
geometry_msgs/Vector3[] arm_eigen_vectors

geometry_msgs/Vector3 hand_centroid
geometry_msgs/Vector3 hand_eigen_value
geometry_msgs/Vector3[] hand_eigen_vectors
geometry_msgs/Point[] fingers

#Possibilities for state variable:
# open - open palm, usually five fingers
# grip - fingers curled forward
# paddle -  fingers together and straight
# fist   
string state
