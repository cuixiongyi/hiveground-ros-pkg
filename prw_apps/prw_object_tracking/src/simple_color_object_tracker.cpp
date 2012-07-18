#include <prw_object_tracking/object_tracking_utils.h>

using namespace prw;

SimpleColorObjectTracker::SimpleColorObjectTracker()
{

}

SimpleColorObjectTracker::~SimpleColorObjectTracker()
{

}

void SimpleColorObjectTracker::setInput(const cv::Mat& input_image)
{

}

void SimpleColorObjectTracker::update()
{

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

void SimpleColorObjectTracker::setColor(const cv::Scalar& hsv_max, const cv::Scalar& hsv_min)
{
  model_.hsv_max_ = hsv_max;
  model_.hav_min_ = hsv_min;
}

void SimpleColorObjectTracker::setSize(int max_size, int min_size)
{
  model_.max_size_ = max_size;
  model_.min_size_ = min_size;
}
