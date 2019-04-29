TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /opt/ros/kinetic/include \
               /usr/include/eigen3 \
               /usr/include/boost \
               /usr/local/include/pcl-1.8 \
               /usr/local/include/pcl-1.8/pcl/io \
               /usr/include/x86_64-linux-gnu \
               /usr/local/include/pcl-1.8/pcl/visualization/vtk \
               /usr/include/vtk-6.2 \
               /opt/pylon5/include \
               /opt/pylon5/include/pylon \

LIBS+= \
      -L/usr/local/lib \
      -lpcl_common \
      -lpcl_features \
      -lpcl_io \
      -lpcl_visualization \
      -lpcl_kdtree \
      -lpcl_keypoints \
      -lpcl_ml \
      -lpcl_octree \
#      -lpcl_outo_fcore \
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
      -lvtkCommonDataModel-6.2 \
      -lvtkCommonCore-6.2 \
      -lvtkRenderingCore-6.2 \
      -lvtkCommonMath-6.2 \
       -L/opt/pylon5/lib64 \
       -lbxapi-5.0.12 \
       -lbxapi \
       -lFirmwareUpdate_gcc_v3_0_Basler_pylon_v5_0 \
       -lGCBase_gcc_v3_0_Basler_pylon_v5_0 \
       -lGenApi_gcc_v3_0_Basler_pylon_v5_0 \
       -lgxapi-5.0.12 \
       -lgxapi \
       -llog4cpp_gcc_v3_0_Basler_pylon_v5_0 \
       -lLog_gcc_v3_0_Basler_pylon_v5_0 \
       -lMathParser_gcc_v3_0_Basler_pylon_v5_0 \
       -lNodeMapData_gcc_v3_0_Basler_pylon_v5_0 \
       -lpylonbase-5.0.12 \
       -lpylonbase \
       -lpylonc-5.0.12 \
       -lpylonc \
       -lpylon_TL_bcon-5.0.12 \
       -lpylon_TL_bcon \
       -lpylon_TL_camemu-5.0.12 \
       -lpylon_TL_camemu \
       -lpylon_TL_gige-5.0.12 \
       -lpylon_TL_gige \
       -lpylon_TL_usb-5.0.12 \
       -lpylon_TL_usb \
       -lpylonutility-5.0.12 \
       -lpylonutility \
       -luxapi-5.0.12 \
       -luxapi \
       -lXmlParser_gcc_v3_0_Basler_pylon_v5_0 \
