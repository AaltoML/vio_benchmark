# Builds DBoW2 and g2o that are dependecies for ORB_SLAM3

set -e

echo "Configuring and building Thirdparty/DBoW2 ..."

cd ORB_SLAM3/Thirdparty/DBoW2
mkdir -p build
cd build
if [[ "$OSTYPE" == "darwin"* ]]; then # Mac
    # OpenCV_DIR to select 3.x instead of 4.x which is default nowdays. -I flag to find boost.
    OpenCV_DIR="/usr/local/opt/opencv@3/" cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-I /usr/local/include"
else # Linux
    cmake .. -DCMAKE_BUILD_TYPE=Release
fi
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
if [[ "$OSTYPE" == "darwin"* ]]; then # Mac
    OpenCV_DIR="/usr/local/opt/opencv@3/" cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-I /usr/local/include"
else # Linux
    cmake .. -DCMAKE_BUILD_TYPE=Release
fi
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ../..
