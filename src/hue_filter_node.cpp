

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

  
  void on_image(const sensor_msgs::ImageConstPtr& msg) {
    bool display = img_pub.getNumSubscribers() > 0;
    bool filter  = filter_pub.getNumSubscribers() > 0;

    if(!filter && !display)
      return;

    cv_bridge::CvImageConstPtr bridge_input;
    try {
      bridge_input = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::RGB8);
    }
    catch (cv::Exception& e) {
      std::ostringstream errstr;
      errstr << "cv_bridge exception caught: " << e.what();
      return;
    }
    
    const cv::Mat& input = bridge_input->image;
    unsigned char* iit   = input.data;
    
    cv::Mat output;
    cv::Mat filtered;
    unsigned char* dit;
    unsigned char* fit;
    
    if(display) {
      output.create(input.rows, input.cols, CV_8UC3);
      dit  = output.data;
    }
    if(filter) {
      filtered.create(input.rows, input.cols, CV_8UC1);
      dit  = filtered.data;
    }
    

    if(display) {

      std::copy(iit, iit + input.rows*input.cols*3, dit);
      img_pub.publish(cv_bridge::CvImage(msg->header, "rgb8", output).toImageMsg());
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
