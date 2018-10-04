---
layout: post
title: "Use Aruco to estimate pose"
date: 2018-10-01 15:14:32 +0800
comments: true
categories: 
---
## Download Aruco library

[Download Aruco library](https://sourceforge.net/projects/aruco/files/3.0.0/aruco-3.0.6.zip/download)

## Build Aruco library

```
cd aruco-3.0.6
mkdir build
cd  build 
cmake .. 
sudo make install
```

## Create map 

```
cd /usr/local/bin/
./aruco_create_markermap 1:3 /home/skye/image.png /home/skye/config.yml -d TAG16h5

```

After printing out the map, measure the length of the marker, suppose the marker size is 0.127 meter, then 

```
./aruco_markermap_pix2meters /home/skye/config.yml 0.127 /home/skye/config_meter.yml 

```

if you get the following error libaruco.so.3.0: cannot open shared object file: No such file or directory

``` 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

```
