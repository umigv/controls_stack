# Note that this is not the original script from the cartographer authors
# This script has been edited to work with a newer version of protobuf 
# (specifically 3.6.1 in this case). Please make sure the protoc version
# is the same for both the apt-get version and the source version. More 
# information about this can be found in the controls_stack github readme
# file. (https://github.com/umigv/controls_stack) -Krishna Dihora

set -o errexit
set -o verbose

VERSION="v3.6.1"

cd ~/catkin_ws
rm -rf protobuf
git clone https://github.com/google/protobuf.git
cd protobuf
git checkout tags/${VERSION}
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
sudo ninja install