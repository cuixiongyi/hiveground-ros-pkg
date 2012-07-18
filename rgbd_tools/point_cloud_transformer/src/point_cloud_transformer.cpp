/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: extract_clusters.hpp 32052 2010-08-27 02:19:30Z rusu $
 *
 */

#include <pluginlib/class_list_macros.h>
#include <pcl/io/io.h>
#include <point_cloud_transformer/point_cloud_transformer.h>

static double xyz_[3] = {-0.7414378772801097, -0.6604714858578639, 1.866349629472909};
static double rpy_[3] = {-0.002052093178870036, 0.1134341257555175, 0.01998161737785098};
tf::Transform tf_;
Eigen::Matrix4f tf_matrix_;

//////////////////////////////////////////////////////////////////////////////////////////////
void hg_pcl::PointCloundTransformer::onInit()
{
  // Call the super onInit ()
  pnh_.reset(new ros::NodeHandle(getMTPrivateNodeHandle()));

  // output
  pub_output_ = pnh_->advertise<PointCloud2> ("output", max_queue_size_);

  // Enable the dynamic reconfigure service
  srv_ = boost::make_shared<dynamic_reconfigure::Server<PointCloudTransformerConfig> >(*pnh_);
  dynamic_reconfigure::Server<PointCloudTransformerConfig>::CallbackType f =
      boost::bind(&PointCloundTransformer::config_callback, this, _1, _2);
  srv_->setCallback(f);


  sub_input_ = pnh_->subscribe<PointCloud2> ("input", max_queue_size_, bind(&PointCloundTransformer::inputCallback, this, _1));

  tf::Quaternion qt = tf::createQuaternionFromRPY(rpy_[0], rpy_[1], rpy_[2]);
  tf::Vector3 vec(xyz_[0], xyz_[1], xyz_[2]);
  qt *= tf::createQuaternionFromRPY(M_PI, 0, M_PI/2.0);
  tf_ = tf::Transform(qt, vec);

  ROS_INFO("%f %f %f : %f %f %f %f\n",
             tf_.getOrigin().x(),
             tf_.getOrigin().y(),
             tf_.getOrigin().z(),
             tf_.getRotation().x(),
             tf_.getRotation().y(),
             tf_.getRotation().z(),
             tf_.getRotation().w());

  tf_ = tf_.inverse();

  ROS_INFO("%f %f %f : %f %f %f %f\n",
           tf_.getOrigin().x(),
           tf_.getOrigin().y(),
           tf_.getOrigin().z(),
           tf_.getRotation().x(),
           tf_.getRotation().y(),
           tf_.getRotation().z(),
           tf_.getRotation().w());

  pcl_ros::transformAsMatrix(tf_, tf_matrix_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void hg_pcl::PointCloundTransformer::config_callback(point_cloud_transformer::PointCloudTransformerConfig &config,
                                                      uint32_t level)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void hg_pcl::PointCloundTransformer::inputCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //NODELET_INFO_STREAM_THROTTLE(1.0, __FUNCTION__);

  // No subscribers, no work
  if (pub_output_.getNumSubscribers() <= 0)
    return;

  sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2 ());
  pcl_ros::transformPointCloud(tf_matrix_, *msg, *cloud_transformed);

  pub_output_.publish(cloud_transformed);

}

typedef hg_pcl::PointCloundTransformer PointCloundTransformer;
PLUGINLIB_DECLARE_CLASS (hg_pcl, PointCloundTransformer, PointCloundTransformer, nodelet::Nodelet);



