#!/bin/sh

git clone https://github.com/Microsoft/AirSim.git

cd AirSim

./setup.sh

./build.sh

echo "AirSim Setup is Done"

pip install numpy

pip install airsim

pip install msgpack-rpc-python

echo "AirSim installation is Done"
