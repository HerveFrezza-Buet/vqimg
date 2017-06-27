

#include <iterator>
#include <algorithm>
#include <utility>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>

#include <vqimg/HueParamConfig.h>

#define HSV_LINE_THICKNESS      20
#define HSV_LINE_CURSOR_RADIUS   1
#define REMAINING_INTENSITY_DISPLAY_RATIO .2

class Params {

public:

  double hue = 0;
  double min_sat = .5;
  double min_val = .5;

public:


  void on_reconf(vqimg::HueParamConfig &config, uint32_t level) {
    hue     = config.hue;
    min_sat = config.min_sat;
    min_val = config.min_val;
  }
};

class Algo {

  Params          params;
  ros::NodeHandle n;
		
  image_transport::ImageTransport it;
  image_transport::Publisher      img_pub;
  image_transport::Publisher      filter_pub;
  image_transport::Subscriber     img_sub;
  
  dynamic_reconfigure::Server<vqimg::HueParamConfig> server;

  cv::Mat hsv_line;
  

public:
  Algo()
    : params(),
      n(),
      it(n),
      img_pub(it.advertise("/image_out", 1)),
      filter_pub(it.advertise("/filter_out", 1)),
      img_sub(it.subscribe("/image_in", 1, boost::bind(&Algo::on_image, boost::ref(*this), _1))),
      server() {
    server.setCallback(boost::bind(&Params::on_reconf, boost::ref(params), _1, _2));
  }

private:

  void check_line(int cols) {
    if(cols == hsv_line.cols)
      return;

    hsv_line.create(1, cols, CV_8UC3);

    unsigned char* it  = hsv_line.data;
    unsigned char* end = hsv_line.data + 3*cols;
    int col = 0;
    while(it != end) {
      *(it++) = (unsigned char)(180.0*(col++)/(cols-1)+.5); // H
      *(it++) = 255;                                        // S
      *(it++) = 255;                                        // V
    }
    cv::cvtColor(hsv_line, hsv_line, CV_HSV2BGR);
  }

  static unsigned char apply_filter(unsigned char* hsv, unsigned char hue, unsigned char min_sat, unsigned char min_val) {
    if(hsv[1] < min_sat || hsv[2] < min_val)
      return 0;
    unsigned char diff;
    if(hue > *hsv)
      diff = hue - *hsv;
    else
      diff = *hsv - hue;
    return 255 - diff;
  }
  
  void on_image(const sensor_msgs::ImageConstPtr& msg) {
    bool display = img_pub.getNumSubscribers() > 0;
    bool filter  = filter_pub.getNumSubscribers() > 0;

    if(!filter && !display)
      return;

    cv_bridge::CvImageConstPtr bridge_input;
    try {
      bridge_input = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv::Exception& e) {
      std::ostringstream errstr;
      errstr << "cv_bridge exception caught: " << e.what();
      return;
    }
    
    const cv::Mat& input = bridge_input->image;
    unsigned char* iit   = input.data;
    unsigned int stride = input.cols*3;
    unsigned int size   = input.rows*stride;
    
    cv::Mat output;
    unsigned char* dit;
    
    if(display) {
      output.create(input.rows, input.cols, CV_8UC3);
      dit  = output.data;
      check_line(input.cols);
    }

    cv::Mat filtered;
    filtered.create(input.rows, input.cols, CV_8UC1);
    unsigned char* fit = filtered.data;
    
    cv::Mat hsv;
    cv::cvtColor(input, hsv, CV_HSV2BGR);
    
    unsigned char* f    = fit;
    unsigned char* hit  = hsv.data;
    unsigned char* hend = hsv.data + size;
    unsigned char min_sat = (unsigned char)(255*params.min_sat + .5);
    unsigned char min_val = (unsigned char)(255*params.min_val + .5);
    unsigned char hue     = (unsigned char)(180*params.hue     + .5);
    for(; hit != hend; hit += 3)
      *(f++) = apply_filter(hit, hue, min_sat, min_val);
      
    
    if(filter)
      filter_pub.publish(cv_bridge::CvImage(msg->header, "mono8", filtered).toImageMsg());

    if(display) {
      std::copy(iit, iit + size, dit);

      unsigned char* ddit = dit;
      unsigned char* dend = dit + size;
      unsigned char* f    = fit;
      double coef = (1-REMAINING_INTENSITY_DISPLAY_RATIO)/255.0;

      while(ddit != dend) {
	double ratio  = *(f++)*coef + REMAINING_INTENSITY_DISPLAY_RATIO;
	*ddit = (unsigned char)(*ddit * ratio); ++ddit;
	*ddit = (unsigned char)(*ddit * ratio); ++ddit;
	*ddit = (unsigned char)(*ddit * ratio); ++ddit;
      }
      
      
      ddit = dit;
      unsigned thick_limit = std::min(input.rows, HSV_LINE_THICKNESS);
      for(unsigned int thick = 0; thick < thick_limit; ++thick, ddit += stride)
	 std::copy(hsv_line.data, hsv_line.data + stride, ddit);

      int pos_min = (int)(params.hue*(input.cols-1)+.5)-HSV_LINE_CURSOR_RADIUS;
      int pos_max = pos_min+2*HSV_LINE_CURSOR_RADIUS;
      cv::rectangle(output, cv::Point(pos_min, 0), cv::Point(pos_max, HSV_LINE_THICKNESS), cv::Scalar(0,0,0), -1);
      
      img_pub.publish(cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg());
    }
  }
};



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hue_node");

  Algo algo;

  ros::spin();


  return 0;
}
