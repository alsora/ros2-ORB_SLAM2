echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2

rm -rf build
mkdir build
cd build

cmake .. \
  -DROS_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
  -DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"


make -j


