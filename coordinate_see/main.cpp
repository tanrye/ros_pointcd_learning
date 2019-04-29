#include <pcl/io/hdl_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vlp_grabber.h>   //这里vlp_grabber tihuan hdl_grabber.h

using namespace std;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;  //POintCloudT shi PointXYZRGBA

boost::mutex cloud_mutex;// Mutex: //

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)//hui diao shi shi jian,chufa huidiao hanshu
{
    struct callback_args* data = (struct callback_args *)args;

    if (event.getPointIndex() == -1)
        return;

    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);

    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    std::vector<pcl::PointIndices> pointIndices;
    pcl::PointIndices::Ptr point_indice (new (pcl::PointIndices));

   // pointIndices.push_back(current_point);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);//custom color for obj

    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");

    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clicked_points");

    std::cout << "x " <<current_point.x << "  y " << current_point.y << "  z " << current_point.z <<"  d "<<sqrt(current_point.x*current_point.x+current_point.y*current_point.y+current_point.z*current_point.z)<<std::endl;
}
/*
float distance(size_t i)
{
    float d=sqrt(cloud->points[i].x*cloud->points[i].x+cloud->points[i].y*cloud->points[i].y+cloud->points[i].z*cloud->points[i].z);
    return d;
}*/

int main()
{
    std::string filename("PcdCol.pcd");                //定义读取变量filename并初始化
    //visualizer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));  //ding yi cheng quanju gongxiang zhizhen

    //pcl::PointCloud<pcl::PointWithRange>            //是存储点云的XYZ和距离信息的类型
    //PointCloud::PointSurfel<pcl::PointSurfel>;

    if (pcl::io::loadPCDFile(filename, *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
        return -1;
    }

    std::cout << "w "<<cloud->width*cloud->height
              <<"  pointsize "<<cloud->points.size()<<std::endl;//<< cloud->is_dense=1 ???

/*------------------------------------------------------------------------------------------------------------*/
    std::vector <float> New_cloud;

/*------------------------------------------------------------------------------------------------------------*/
    float kl=0.0,kr=0.0;

    cloud_mutex.lock();

    viewer->addPointCloud(cloud, "bunny");

    //viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);

    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);

    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    viewer->spin();//gengxin xianshi
    std::cout << "done." << std::endl;

    cloud_mutex.unlock();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));//100000haomiao ;second_clock: biaoshi miao;mill:weimiao
    }
}

/*---ke shi hua yong de shi PCLVisualization lei--------*/
