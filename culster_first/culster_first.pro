TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
DESTDIR = ./build
OBJECTS_DIR = ./build/tmp

SOURCES += main.cpp

INCLUDEPATH += /opt/ros/kinetic/include \
               /usr/include/eigen3 \
               /usr/include/boost \
               /usr/local/include/pcl-1.8 \
               /usr/include/x86_64-linux-gnu \
               /usr/local/include/pcl-1.8/pcl/visualization/vtk \
               /usr/include/vtk-6.2 \


LIBS+= \
      -L/usr/local/lib \
      -lpcl_common \
      -lpcl_features \
      -lpcl_filters \
      -lpcl_io \
      -lpcl_visualization \
      -lpcl_kdtree \
      -lpcl_keypoints \
      -lpcl_ml \
      -lpcl_octree \
      -lpcl_people \
      -lpcl_recognition \
      -lpcl_registration \
      -lpcl_sample_consensus \
      -lpcl_search \
      -lpcl_surface \
      -lpcl_tracking \
      -lpcl_stereo \
      -lpcl_segmentation \
      -L/usr/lib/x86_64-linux-gnu \
      -lboost_system \
      -lboost_thread \
      -lpthread \
      -lvtkRenderingLOD-6.2\
      -lvtkCommonDataModel-6.2 \
      -lvtkCommonCore-6.2 \
      -lvtkRenderingCore-6.2 \
      -lvtkCommonMath-6.2 \
      -L/opt/ros/kinetic/lib \
       -lroscpp \
       -lrosconsole \
       -lroscpp_serialization \
       -lrostime \
       -lxmlrpcpp \
       -lcpp_common \
       -lrosconsole_log4cxx \
       -lrosconsole_backend_interface \
       -lboost_system \
       -lboost_thread \
       -lpthread \
