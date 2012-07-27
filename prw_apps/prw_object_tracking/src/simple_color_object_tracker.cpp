#include <prw_object_tracking/object_tracking_utils.h>
#include <QDebug>

using namespace prw;
using namespace cv;

SimpleColorObjectTracker::SimpleColorObjectTracker()
  : found_in_last_image_(false)
{

}

SimpleColorObjectTracker::~SimpleColorObjectTracker()
{

}

void SimpleColorObjectTracker::setInput(const cv::Mat& input_image)
{
  last_image_ = current_image_;
  current_image_ = input_image;
}

void SimpleColorObjectTracker::update()
{

  if(model_.min_hsv_.val[0] > model_.max_hsv_.val[0])
  {
    cv::Mat mask1, mask2;
    cv::Scalar upper = model_.max_hsv_;
    cv::Scalar lower = model_.min_hsv_;
    upper.val[0] = 180;
    lower.val[0] = 0;
    inRange(current_image_, model_.min_hsv_, upper, mask1);
    inRange(current_image_, lower, model_.max_hsv_, mask2);
    bitwise_or(mask1, mask2, mask_);
  }
  else
  {
    inRange(current_image_, model_.min_hsv_, model_.max_hsv_, mask_);
  }



  cv::Mat filtered;
  cv::erode(mask_, filtered, cv::Mat());
  cv::dilate(filtered, mask_, cv::Mat());

  //cv::imshow("mask", mask_);

  std::vector<std::vector<cv::Point> > contours;
  findContours(mask_, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  /*
  for(int i = 0; i < contours.size(); i++)
  {
    qDebug() << i << ":" << cv::contourArea(contours[i]);
  }
  */

  contours_filtered_.clear();
  Mat dst = Mat::zeros(mask_.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]);
    if(area <= model_.max_size_ && area >= model_.min_size_)
    {
      contours_filtered_.push_back(contours[i]);
      //qDebug() << area << contours_filtered.back().size();
    }
  }

  if(contours_filtered_.size() != 0)
  {
    //drawContours(dst, contours_filtered_, -1, model_.max_hsv_, 1, 8);
    found_in_last_image_ = true;
  }
  else
  {
    found_in_last_image_  = false;
  }

  /*
  if( !contours.empty() && !hierarchy.empty() )
  {
    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    for (; idx >= 0; idx = hierarchy[idx][0])
    {
      if(cv::contourArea(contours);)
      drawContours(dst, contours, idx, model_.max_hsv_, 1, 8, hierarchy);
    }
  }
  */
  //cv::imshow("dst", dst);


}

bool SimpleColorObjectTracker::getResult(cv::Mat& result)
{
  return false;
}

bool SimpleColorObjectTracker::getResult(std::vector<cv::RotatedRect>& result)
{
  result.clear();
  if(!found_in_last_image_)
    return false;

  for(int i = 0; i < contours_filtered_.size(); i++)
  {
    if(contours_filtered_[i].size() > 5)
      result.push_back(cv::fitEllipse(contours_filtered_[i]));
  }
}

bool SimpleColorObjectTracker::load(const std::string& file)
{
  return false;
}

bool SimpleColorObjectTracker::save(const std::string& file)
{
  return false;
}

void SimpleColorObjectTracker::setColor(const cv::Scalar& hsv_min, const cv::Scalar& hsv_max)
{
  model_.min_hsv_ = hsv_min;
  model_.max_hsv_ = hsv_max;
}

void SimpleColorObjectTracker::setSize(int min_size, int max_size)
{
  model_.min_size_ = min_size;
  model_.max_size_ = max_size;
}
