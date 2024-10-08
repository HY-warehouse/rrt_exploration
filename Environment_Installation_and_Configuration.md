# Environment Installation and Configuration

## Install Ubuntu 18.04/20.04
Due to the need to use a GPU, it is recommended to use a dual-boot system.



## Install ROS Melodic/Noetic

If you want convenience, you can use FishROS’s one-line installation tool, which will automatically help you change sources and install ROS.
```bash
wget http://fishros.com/install -O fishros && . Fishros
```
After copying the above command into the terminal, select the option to install ROS based on the printed content.



## Downhill and Configure YOLOv5

### Install NVIDIA driver
1. Use
```bash
sudo apt install ubuntu-drivers-common
ubuntu-drivers devices
```
to obtain the NVIDIA graphics card model and recommended drivers.

2. Select the appropriate driver version, using 525 as an example, and enter the following commands in the terminal.
```bash
sudo apt install nvidia-driver-525
```

### Install CUDA 11.6 and CUDNN
The official address for CUDA and CUDNN installation is as follows.
[CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive)
[CUDA Deep Neural Network](https://developer.nvidia.com/cudnn)

### Install Anaconda
Offcial address:
[Anaconda](https://www.anaconda.com/products/distribution#Downloads)



## Install QT5 and Configure FFmpeg

### Install QT5-linux
Go to the Tsinghua mirror site: [QT5-linux](https://mirrors.tuna.tsinghua.edu.cn/qt/archive/qt/) . Select Qt5.12.9
Then run 
```bash
sudo ./qt-opensource-linux-x64-5.12.9.run
```

### Install the required libraries
```bash
sudo apt-get install build-essential
sudo apt-get install build-essential libgl1-mesa-dev
sudo apt-get install libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev
sudo apt-get install libglfw3-dev libglfw3
```

### Install FFmpeg
We have chosen version 2.8.17 for installation.

Official website: [http://ffmpeg.org/releases/](http://ffmpeg.org/releases/)

After downloading, extract the files:
```bash
sudo tar -zxvf ffmpeg-2.8.17.orig.tar.xz
```

The configuration command for `configure` is:
```bash
sudo ./configure --enable-gpl --enable-libx264 --prefix=/usr/local/ffmpeg-build --enable-shared
```

- `gpl` and `libx264` are additional libraries that can be selected as needed; they are disabled by default.
- `--prefix`: Specifies the installation path. If not specified, header files will be installed in `/usr/local/include` by default, and libraries will be in `/usr/local/lib`.
- `--enable-shared`: Chooses to compile dynamic libraries. Without this option, only static libraries (`.a`) will be compiled. Adding this option will also compile dynamic libraries (`.so`).

After configuring, execute the compile and install commands:
```bash
make -j8 && sudo make install
```

Wait for a moment, and it will be done.


### Adding FFmpeg to the Environment

To add FFmpeg to the environment variables, first execute:
```bash
sudo gedit /etc/profile
```

At the end of the file, add the path to the FFmpeg main program and library:
```bash
export PATH=$PATH:/usr/local/ffmpeg-build/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/ffmpeg-build/lib
```

After saving, execute:
```bash
source /etc/profile
```

In the Qt `.pro` file, import FFmpeg:
```qmake
INCLUDEPATH += /usr/local/ffmpeg-build/include
LIBS += /usr/local/ffmpeg-build/lib/libavformat.so \
        /usr/local/ffmpeg-build/lib/libavdevice.so \
        /usr/local/ffmpeg-build/lib/libavcodec.so \
        /usr/local/ffmpeg-build/lib/libavfilter.so \
        /usr/local/ffmpeg-build/lib/libavutil.so    \
        /usr/local/ffmpeg-build/lib/libswscale.so \
        /usr/local/ffmpeg-build/lib/libswresample.so
```
