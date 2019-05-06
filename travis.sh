#!/bin/bash


# Run Docker container
if ! [ "$IN_DOCKER" ]; then

  docker pull $DOCKER_IMAGE

  ci_env=`bash <(curl -s https://codecov.io/env)`

  docker run \
    $ci_env \
    -e IN_DOCKER=true \
    -e TRAVIS_BRANCH \
    -e TRAVIS_BUILD_DIR \
    -v $(pwd):/root/$(basename $PWD) \
    -v $HOME/.ccache:/root/.ccache \
    -w /root/$(basename $PWD) \
    -t \
    $DOCKER_IMAGE /root/$(basename $PWD)/./$SCRIPT
  result=$?

  case $result in
    0) tput setaf 2; echo "Travis script finished successfully";;
    1) tput setaf 1; echo "Travis script failed at catkin_lint";;
    2) tput setaf 1; echo "Travis script failed at catkin_make_isolated";;
    3) tput setaf 1; echo "Travis script failed at catkin_make run_tests";;
    4) tput setaf 1; echo "Travis script failed at catkin_test_results";;
  esac
  exit $result
fi


# Display system information
echo "##############################################"
uname -a
lsb_release -a
gcc --version
echo "CXXFLAGS = ${CXXFLAGS}"
cmake --version
echo "##############################################"


# Setup ROS
source /opt/ros/$(ls /opt/ros/)/setup.bash
export CCACHE_DIR=/root/ccache

# Prepare workspace
PROJECT_NAME="testing"
URL=${TRAVIS_BUILD_DIR/"/home/travis/build"/"https://github.com"}
#cd /root
mkdir -p /catkin_ws/src
git clone $URL -b $TRAVIS_BRANCH /catkin_ws/src/$PROJECT_NAME


# Initialize git submodules
cd /catkin_ws/src/$PROJECT_NAME
git submodule update --init --recursive
cd ../..


# Lint
catkin_lint -W3 . #|| exit 1

# Make
catkin_make_isolated #|| exit 2

# Test
catkin_make run_tests #|| exit 3
catkin_test_results #|| exit 4

# Code coverage
catkin_make -DCMAKE_BUILD_TYPE=Coverage toposens_driver_coverage #|| exit 5

#cd build
#ls -al
#cd coverage
#ls -al
#echo "$(cat toposens_driver.info)"

#cd ../../..

#echo "--"
#lcov --list toposens_driver.info
#echo "---"
#lcov --list toposens_driver.info.cleaned
#echo "----"

#echo $PWD
#echo $CODECOV_TOKEN
#CODECOV_TOKEN="39fdfe66-5f8c-468e-b68f-4d6529702b14"
#bash <(curl -s https://codecov.io/bash) -X gcov -s catkin_ws/build/coverage -f toposens_driver.info -R ycatkin_ws/src/ts-ros -v
#bash <(curl -s https://codecov.io/bash) -f toposens_driver.info


#cd ../..
mkdir -p lcov
ls -a
#cd lcov
lcov --directory . --capture --output-file coverage.info
lcov --list coverage.info
lcov --remove coverage.info '/src/ts-ros/toposens_driver/src/lib/*' --output-file coverage.info
ls -a
lcov --list coverage.info
bash <(curl -s https://codecov.io/bash) -X gcov || echo "Codecov did not collect coverage reports"
