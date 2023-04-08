cd ~ 
git clone https://github.com/IntelRealSense/librealsense.git
sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev at 
./scripts/setup_udev_rules.sh
mkdir build
cd build 

echo 'export CUDA_HOME=/usr/locoal/cuda' >> ~/.bashrc 
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/cuda/extras/CUPTI/lib64' > ~/.bashrc
echo 'export PATH=$PATH:$CUDA_HOME/bin' >> ~/.bashrc
cmake .. --DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true && make -j(($(nproc)-1 )) && sudo make install 
