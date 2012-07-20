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
  inRange(current_image_, model_.hsv_min_, model_.hsv_max_, mask_);
  cv::Mat filtered;
  cv::erode(mask_, filtered, cv::Mat());
  cv::dilate(filtered, mask_, cv::Mat());

  cv::imshow("mask", mask_);

  /*
  cv::Mat hue;
  int ch[] = {0, 0};
  hue.create(current_image_.size(), current_image_.depth());
  cv::mixChannels(&current_image_, 1, &hue, 1, ch, 1);

  cv::imshow("hue", hue);
  */

  std::vector<std::vector<cv::Point> > contours;
  vector<Vec4i> hierarchy;
  //findContours(mask_, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  findContours(mask_, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  /*
  for(int i = 0; i < contours.size(); i++)
  {
    qDebug() << i << ":" << cv::contourArea(contours[i]);
  }
  */

  std::vector<std::vector<cv::Point> > contours_filtered;
  Mat dst = Mat::zeros(mask_.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]);
    if(area <= model_.max_size_ && area >= model_.min_size_)
    {
      contours_filtered.push_back(contours[i]);
    }
  }

  drawContours(dst, contours_filtered, -1, model_.hsv_max_, 1, 8);

  /*
  if( !contours.empty() && !hierarchy.empty() )
  {
    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    for (; idx >= 0; idx = hierarchy[idx][0])
    {
      if(cv::contourArea(contours);)
      drawContours(dst, contours, idx, model_.hsv_max_, 1, 8, hierarchy);
    }
  }
  */
  cv::imshow("dst", dst);


}

bool SimpleColorObjectTracker::getResult(cv::Mat& result)
{
  return false;
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
  model_.hsv_min_ = hsv_min;
  model_.hsv_max_ = hsv_max;
}

void SimpleColorObjectTracker::setSize(int min_size, int max_size)
{
  model_.min_size_ = min_size;
  model_.max_size_ = max_size;
}
