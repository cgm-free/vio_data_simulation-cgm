#!/bin/bash
echo "Configuring and building ..."
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
cd ../bin
./data_gen
cd ../python_tool
python3 draw_points.py