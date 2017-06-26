


#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <vq2.h>
#include <opencv2/opencv.hpp>

#include <vqimg/GngtParamConfig.h>
#include <iterator>
#include <map>
#include <algorithm>

class Params {

public:

  unsigned char thresh = 127;
  
  double lr  = 0.01;
  double lrr = 0.2;
  double t   = 1;
  unsigned int max_samples = 1000;  
  double       nb_samples = 1; 
  unsigned int nb_epochs = 1;

  unsigned int width = 0;
  unsigned int height = 0;
  unsigned int img_size = 0;

  double max_dist = .1;

public:

  void check_img(unsigned int w, unsigned int h) {
    if(w != width || h != height) {
      width = w;
      height = h;
      img_size = w*h;
    }
  }
  
  unsigned int update_nb_samples(unsigned int data_size) {
    auto all = width*height;
    
    if(data_size < max_samples) {
      nb_samples = 1;
      return data_size;
    }
    else {
      nb_samples = max_samples/(double)img_size;
      return max_samples;
    }
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


  void on_reconf(vqimg::GngtParamConfig &config, uint32_t level) {
    thresh = (unsigned char)(config.threshold);
    nb_epochs = (unsigned int)(config.nb_epochs);
    t = config.target;
    max_samples = (unsigned int)(config.max_samples);
    max_dist = config.max_dist;
  }
};

class Algo {
  typedef vq2::algo::gngt::Unit<cv::Point2d>              Unit;
  typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;
  typedef Graph::vertex_type                              Vertex;
  typedef Graph::edge_type                                Edge;
  typedef Graph::ref_vertex_type                          RefVertex;
  typedef Graph::ref_edge_type                            RefEdge;
  typedef vq2::by_default::gngt::Evolution<Params>        Evolution;

  class LabelToColor {
  private:

    std::map<unsigned int, cv::Scalar> colormap;
    
  public:
    
    LabelToColor() {}
    cv::Scalar operator()(unsigned int label) {
      auto iter = colormap.find(label);
      if(iter != colormap.end())
	return iter->second;
      else {
	double r = vq2::proba::random::uniform(0,255);
	double g = vq2::proba::random::uniform(0,255);
	double b = vq2::proba::random::uniform(0,255);

	double min = std::min({r, g, b});
	double max = std::max({r, g, b})+1e-3;
	double coef = 255.0/(max-min);

	cv::Scalar color((r-min)*coef, (g-min)*coef, (b-min)*coef);
	colormap[label] = color;
	return color;
      }
    }
  };


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
  typedef vq2::unit::Learn<Unit,Learn> UnitLearn;

  Params          params;
  ros::NodeHandle n;
		
  image_transport::ImageTransport it;
  image_transport::Publisher      img_pub;
  image_transport::Subscriber     img_sub;
  
  dynamic_reconfigure::Server<vqimg::GngtParamConfig> server;
  
  Graph            g;
  Similarity       distance;
  UnitSimilarity   unit_distance;
  Learn            learn;
  UnitLearn        unit_learn;
  Evolution        evolution;

  std::vector<cv::Point2d> samples;

  class ReshuffleSamples {
  private:
    std::vector<cv::Point2d>& samples;
    unsigned int size;
  
  public:
    ReshuffleSamples(std::vector<cv::Point2d>& s, unsigned int size) : samples(s), size(size) {}
    std::vector<cv::Point2d>::iterator begin(void) {
      std::random_shuffle(samples.begin(),samples.end());
      return samples.begin();
    }
  
    std::vector<cv::Point2d>::iterator end(void) {
      return samples.begin()+size;
    }
  };

  LabelToColor l2c;

public:
  Algo()
    : params(),
      n(),
      it(n),
      img_pub(it.advertise("/image_out", 1)),
      img_sub(it.subscribe("/image_in", 1, boost::bind(&Algo::on_image, boost::ref(*this), _1))),
      server(),
      g(),
      distance(),
      unit_distance(distance),
      learn(),
      unit_learn(learn),
      evolution(params),
      l2c() {
    server.setCallback(boost::bind(&Params::on_reconf, boost::ref(params), _1, _2));
  }

private:

  static cv::Point2d cv2gngt(const cv::Point2d& in, int rows_2, double coef) {
    return cv::Point2d(in.x*coef-.5, (rows_2-in.y)*coef);
  }
  
  static cv::Point2i gngt2cv(const cv::Point2d& in, int rows_2, int cols) {
    return cv::Point2i((int)((in.x+.5)*cols+.5), (int)(-in.y*cols + rows_2+.5));
  }

  friend class DrawVertex;
  class DrawVertex {
  public:
    cv::Mat& img;
    cv::Scalar color;
    int rows_2;
    int cols;
    DrawVertex(cv::Mat& img, int rows_2, int cols) : img(img), color(255,255,255), rows_2(rows_2), cols(cols) {}
    bool operator()(Graph::vertex_type& v) {
      cv::circle(img, Algo::gngt2cv(v.value.prototype(), rows_2, cols), 4, color, -1);
      return false;
    }
  };
  
  friend class DrawEdge;
  class DrawEdge {
  public:
    cv::Mat& img;
    cv::Scalar color;
    int rows_2;
    int cols;
    DrawEdge(cv::Mat& img, int rows_2, int cols) : img(img), color(255,255,255), rows_2(rows_2), cols(cols) {}
    bool operator()(Graph::edge_type& e) {
      cv::line(img,
	       Algo::gngt2cv((*(e.n1)).value.prototype(), rows_2, cols),
	       Algo::gngt2cv((*(e.n2)).value.prototype(), rows_2, cols),
	       color);
      return false;
    }
  };
  
  friend class InvalidateLongEdge;
  class InvalidateLongEdge {
  private:
    double msd;
  public:
    InvalidateLongEdge(double max_dist) : msd(max_dist*max_dist) {}
    
    bool operator()(Graph::edge_type& e) { 
      Algo::Similarity squared_dist;
      auto& A = (*(e.n1)).value.prototype();
      auto& B = (*(e.n2)).value.prototype();
      e.stuff.efficient = squared_dist(A,B) < msd;
      return false; 
    }
  };

  
  
  void on_image(const sensor_msgs::ImageConstPtr& msg) {
    bool display = img_pub.getNumSubscribers() > 0;

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
    
    cv::Mat output;
    unsigned char* dit;
    if(display) {
      output.create(input.rows, input.cols, CV_8UC3);
      dit  = output.data;
    }
    
    auto sit  = input.data;
    cv::Point2d xi;
    samples.clear();
    auto out = std::back_inserter(samples);
    double coef = 1.0/input.cols;
    int rows_2 = input.rows/2;

    if(display)
      for(xi.y = 0; xi.y < input.rows; ++xi.y)
	for(xi.x = 0; xi.x < input.cols; ++xi.x) 
	  if(*(sit++) > params.thresh) {
	    *(dit++) = 128;
	    *(dit++) = 128;
	    *(dit++) = 128;
	    *(out++) = cv2gngt(xi, rows_2, coef);
	  }
	  else {
	    *(dit++) = 0;
	    *(dit++) = 0;
	    *(dit++) = 0;
	  }
    else
      for(xi.y = 0; xi.y < input.rows; ++xi.y)
	for(xi.x = 0; xi.x < input.cols; ++xi.x) 
	  if(*(sit++) > params.thresh)
	    *(out++) = cv2gngt(xi, rows_2, coef);

    ReshuffleSamples reshuffler(samples, params.update_nb_samples(samples.size()));
    vq2::algo::gngt::epoch(params,g,
                           unit_distance,unit_learn,
                           evolution,reshuffler,
                           [] (const cv::Point2d& p) -> const cv::Point2d& {return p;},
                           params.nb_epochs);
    
    InvalidateLongEdge ile(params.max_dist);
    g.for_each_edge(ile);
    std::map<unsigned int,Graph::Component*> components;
    g.computeConnectedComponents(components, true); 

    

    if(display) {
      DrawVertex dw(output, rows_2, input.cols);
      DrawEdge de(output, rows_2, input.cols);
      for(auto& kv : components) {
	de.color = l2c(kv.first);
	dw.color = de.color;
	kv.second->for_each_edge(de);
	kv.second->for_each_vertex(dw);
      }
      img_pub.publish(cv_bridge::CvImage(msg->header, "rgb8", output).toImageMsg());
    }
  }
};



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gngt_node");

  Algo algo;

  ros::spin();


  return 0;
}
