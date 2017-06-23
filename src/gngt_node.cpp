


#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <vq2.h>
#include <opencv2/opencv.hpp>

class Params {

public:

  unsigned char thresh = 127;
  
  double lr  = 0.01;
  double lrr = 0.2;
  double t   = 0.01;
  double r   = 1;    // 1 <=> 100% of the samples
  
  unsigned int width      = 0;
  unsigned int height     = 0;
  unsigned int img_size   = 0;
  double       nb_samples = 1; // r*width*height;

public:

  void check_img(unsigned int w, unsigned int h) {
    if(w != width || h != height) {
      width = w;
      height = h;
      img_size = w*h;
      update_nb_samples();
    }
  }
  
  void update_nb_samples() {
    nb_samples = width*height*r;
  }
  
  // GNG-T
  int    ageMax(void)        const {return 20;}
  double learningRate(void)  const {return lr;}
  double learningRatio(void) const {return lrr;}
  double lambda(void)        const {return .001;}
  
  // Evolution
  double target(void)        const {return t;}
  double nbSamples(void)     const {return nb_samples;}
  double lowPassCoef(void)   const {return .4;}
  double delta(void)         const {return .75;}
  double margin(void)        const {return .2;}
};

class Algo {
  typedef vq2::algo::gngt::Unit<cv::Point2d>              Unit;
  typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;
  typedef Graph::vertex_type                              Vertex;
  typedef Graph::edge_type                                Edge;
  typedef Graph::ref_vertex_type                          RefVertex;
  typedef Graph::ref_edge_type                            RefEdge;


  class Similarity {
  public:
    typedef cv::Point2d value_type;
    typedef cv::Point2d sample_type;

    double operator()(const value_type& arg1,
		      const sample_type& arg2) {
      double dx = arg1.x - arg2.x;
      double dy = arg1.y - arg2.y;
      return dx*dx + dy*dy;
    }
  };
  typedef vq2::unit::Similarity<Unit,Similarity> UnitSimilarity;

  class Learn {
  public:
    typedef cv::Point2d sample_type;
    typedef cv::Point2d weight_type;
    void operator()(double coef,
		    weight_type& prototype,
		    const sample_type& target) {
      prototype.x += coef * (target.x - prototype.x);
      prototype.y += coef * (target.y - prototype.y);
    }
  };

  Params          params;
  ros::NodeHandle n;
		
  image_transport::ImageTransport it;
  image_transport::Publisher      img_pub;
  image_transport::Subscriber     img_sub;

public:
  Algo()
    : n(),
      it(n),
      img_pub(it.advertise("/image_out", 1)),
      img_sub(it.subscribe("/image_in", 1, boost::bind(&Algo::on_image, boost::ref(*this), _1))) {
  }

private:
  
  void on_image(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImageConstPtr bridge_input;
    try {
      bridge_input = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::MONO8);
    }
    catch (cv::Exception& e) {
      std::ostringstream errstr;
      errstr << "cv_bridge exception caught: " << e.what();
      return;
    }
    
    const cv::Mat& input  = bridge_input->image;
    params.check_img(input.rows, input.cols);
    
    cv::Mat output(input.rows, input.cols, CV_8UC3);

    auto sit  = input.data;
    auto send = input.data + params.img_size;
    auto dit  = output.data;
    while(sit != send)
      if(*(sit++) < params.thresh) {*(dit++) =   0; *(dit++) =   0; *(dit++) =   0;}
      else                         {*(dit++) = 128; *(dit++) = 128; *(dit++) = 128;}
    
    img_pub.publish(cv_bridge::CvImage(msg->header, "rgb8", output).toImageMsg());
  }
};



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gngt_node");

  Algo algo;

  ros::spin();


  return 0;
}
