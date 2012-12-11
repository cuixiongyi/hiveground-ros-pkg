#include <prw_object_tracking/object_tracking_utils.h>
#include <QDebug>

using namespace prw;
using namespace cv;


LutColorObjectTraker::LutColorObjectTraker()
  : lut_(boost::extents[H_RANGE][S_RANGE][V_RANGE])
{
  cv::Mat hsv = cv::imread("noise.png");
  if(hsv.empty())
  {
    std::cout << "cannot read image";
    return;
  }

  cv::Mat index(480, 640, CV_8UC1);
  std::vector<Mat> planes;
  split(hsv, planes);


  int channels = hsv.channels();

  int nRows = hsv.rows * channels;
  int nCols = hsv.cols;

  if (hsv.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }


  double t = (double)getTickCount();
  int i, j;
  uchar* p;
  for (i = 0; i < nRows; ++i)
  {
    p = hsv.ptr < uchar > (i);
    for (j = 0; j < nCols; j+=channels)
    {
      p[j] = lut_[p[j]][p[j+1]][p[j+2]];
      //p[j] = 0;
      //p[j+1] = 1;
      //p[j+2] = 2;
    }
  }



  /*
  MatIterator_<Vec3b> it, end;
  MatIterator_<uchar> it_index;
  for (it = hsv.begin<Vec3b>(), end = hsv.end<Vec3b>(), it_index = index.begin<uchar>(); it != end; ++it, ++it_index)
  {

    *it_index = lut_[(*it)[0]][(*it)[1]][(*it)[2]];
    //(*it)[0] = 0;
    //(*it)[1] = 1;
    //(*it)[2] = 2;
  }
  */
  /*
  for(int i = 0; i < hsv.rows; i++)
    for(int j = 0; j < hsv.cols; j++)
    {
      //planes[0].at<unsigned char>(i,j) = 0;
      //planes[1].at<unsigned char>(i,j) = 1;
      //planes[2].at<unsigned char>(i,j) = 2;
    }
  */
  t = ((double)getTickCount() - t)/getTickFrequency();
  std::cout << "time: " << t << std::endl;
}

LutColorObjectTraker::~LutColorObjectTraker()
{

}

void LutColorObjectTraker::setInput(const cv::Mat& input_image)
{
  last_image_ = current_image_;
  current_image_ = input_image;
}

void LutColorObjectTraker::update()
{

}

bool LutColorObjectTraker::getResult(cv::Mat& result)
{
  return false;
}

bool LutColorObjectTraker::getResult(std::vector<cv::RotatedRect>& result)
{
  return false;
}

bool LutColorObjectTraker::load(const std::string& file)
{
  return false;
}

bool LutColorObjectTraker::save(const std::string& file)
{
  return false;
}

void LutColorObjectTraker::addObject(const std::string& name, const cv::Scalar& min_hsv, const cv::Scalar& max_hsv,
                                     int min_size, int max_size)
{
  ColorObject color_object;
  color_object.min_hsv_ = min_hsv;
  color_object.max_hsv_ = max_hsv;
  color_object.min_size_ = min_size;
  color_object.max_size_ = max_size;
  color_object_map_[name] = color_object;
  updateLut();

}
void LutColorObjectTraker::updateObject(const std::string& name, const cv::Scalar& min_hsv, const cv::Scalar& max_hsv,
                                        int min_size, int max_size)
{

}
void LutColorObjectTraker::deleteObject(const std::string& name)
{

}

void LutColorObjectTraker::updateLut()
{
  ColorObjectMap::iterator it;
  int index = 0;

  double t = (double)getTickCount();

  for(LookupTableIndex h = 0; h < H_RANGE; h++)
  {
    for(LookupTableIndex s = 0; s < S_RANGE; s++)
    {
      for(LookupTableIndex v = 0; v < V_RANGE; v++)
      {
        index = lut_[h][s][v];
        /*
        for(it = color_object_map_.begin(), index = 0; it != color_object_map_.end(); it++, index++)
        {
          if((v >= it->second.min_hsv_.val[2] && v <= it->second.max_hsv_.val[2]))
          {
            if((s >= it->second.min_hsv_.val[1] && s <= it->second.max_hsv_.val[1]))
            {
              if(it->second.min_hsv_.val[0] > it->second.max_hsv_.val[0])
              {
                if((h <= H_RANGE && h >= it->second.min_hsv_.val[0]) ||
                   (h >= 0 && h <= it->second.max_hsv_.val[0]))
                {
                  lut_[h][s][v] = index;
                }
              }
              else
              {
                if((h >= it->second.min_hsv_.val[0] && h <= it->second.max_hsv_.val[0]))
                {
                  lut_[h][s][v] = index;
                }
              }//check h
            }//check s
          }//check v
        }
        */
      }
    }
  }

  t = ((double)getTickCount() - t)/getTickFrequency();
  std::cout << "LUT update time: " << t << std::endl;
}
