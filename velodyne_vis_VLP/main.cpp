#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>


using namespace boost;
using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleVLPViewer
{
public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleVLPViewer (pcl::Grabber& grabber,pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
        : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL VLP Cloud")),
       grabber_ (grabber),
       handler_ (handler)
    {
    }

    void cloud_callback (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void run ()
    {
      cloud_viewer_->addCoordinateSystem (0.5);
      cloud_viewer_->setBackgroundColor (0,0,0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (2.0,2.0,0.0,0.0,0.0,0.0,0);
      cloud_viewer_->setCameraClipDistances (0.0,100.0,1);

      function<void (const CloudConstPtr&)> cloud_cb = bind (&SimpleVLPViewer::cloud_callback, this, _1);
      signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
      grabber_.start ();
      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr cloud;

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }/**/

        if (cloud)
        {
          handler_.setInputCloud (cloud);
          if (!cloud_viewer_->updatePointCloud (cloud, handler_, "VLP"))
          cloud_viewer_->addPointCloud (cloud, handler_, "VLP");
            //addPointCloud(source,sources_cloud_color,"sources_cloud_v1",v1);
          cloud_viewer_->spinOnce ();
          std::cout << "PointCloud sum: " << cloud->points.size () << " data points."
                    <<" weight "<<cloud->width<<" height "<<cloud->height<< std::endl;

        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();
          //grabber_.getFramesPerSecond();
        this_thread::sleep (posix_time::microseconds (10));

     }
      grabber_.stop ();

      cloud_connection.disconnect ();

    }

    shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};


int main (int argc, char ** argv)
{
  //string hdlCalibration, pcapFile;
  //typedef VLPGrabber HDLGrabber;

  // parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  // parse_argument (argc, argv, "-pcapFile", pcapFile);

  //VLPGrabber* grabber=new VLPGrabber (pcapFile);
   VLPGrabber grabber;  //grabber(pcapFile)

  PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
  SimpleVLPViewer v (grabber,color_handler);

  v.run ();

  return (0);
}
