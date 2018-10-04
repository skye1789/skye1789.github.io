---
layout: post
title: "aruco posetracker"
date: 2018-10-03 15:23:03 +0800
comments: true
categories:
---

```c++
cv::Mat  MarkerMapPoseTracker::relocalization(const std::vector<Marker>& v_m){


        //get the markers in v_m that are in the map
        std::vector<Marker> mapMarkers;
        for (auto marker : v_m)
        {
            if (_map_mm.find(marker.id) != _map_mm.end())
                mapMarkers.push_back(marker);
        }

        if( mapMarkers.size()==0)return cv::Mat();
        struct minfo{
            int id;
            cv::Mat rt_f2m;
            double err;
        };
        struct se3{float rt[6];};

        cv::Mat pose_f2g_out;//result
        //estimate the markers locations and see if there is at least one good enough
        std::vector<minfo> good_marker_locations;
        std::vector<minfo> all_marker_locations;

        for(const Marker &marker:mapMarkers){//for ech visible marker
             auto mpi=solvePnP_(_map_mm[marker.id]. getMarkerSize(),marker,_cam_params.CameraMatrix,_cam_params.Distorsion);
            minfo mi;
            mi.id=marker.id;
            mi.err=mpi[0].second;
            mi.rt_f2m=mpi[0].first;
            all_marker_locations.push_back(mi);
            if(mpi[1].second/mpi[0].second >  aruco_minerrratio_valid)
                good_marker_locations.push_back(mi);
            mi.rt_f2m=mpi[1].first;
            mi.err=mpi[1].second;
            all_marker_locations.push_back(mi);

        }


        //try using more than one marker approach
        if (mapMarkers.size()>=2) {
            //collect all the markers 3d locations
            std::vector<cv::Point2f> markerPoints2d;
            std::vector<cv::Point3f> markerPoints3d;
	//mapMarkers type: std::vector<Marker>
            for(const   Marker &marker:mapMarkers){
                markerPoints2d.insert(markerPoints2d.end(),marker.begin(),marker.end());
                auto p3d= _map_mm[marker.id].points;
                markerPoints3d.insert(markerPoints3d.end(),p3d.begin(),p3d.end());
            }

```
`cv::projectPoints` takes 3d points $(X,Y,Z,1)$ in global coordinate, R, t and camera matrix K as argument, and return the corresponding 2d points $(u,v,1)$.

$$
Z
\begin{bmatrix}
u \\ v \\ 1
\end{bmatrix}=
\begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 0 & 0\\
0& 1 & 0 & 0\\
0& 0 & 1 & 0\\
\end{bmatrix}
\begin{bmatrix}
R_{11} & R_{12} & R_{13} & t_1\\
R_{21} & R_{22} & R_{23} & t_2\\
R_{31} & R_{32} & R_{33} & t_3\\
0 & 0 & 0 & 1\\
\end{bmatrix}
\begin{bmatrix}
X \\ Y \\ Z \\1
\end{bmatrix}$$

```c++
double reprj_error( const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const CameraParameters &imp,const cv::Mat &rt44){
    std::vector<cv::Point2f> prepj;
     cv::Mat rv,tv;
    impl__aruco_getRTfromMatrix44(rt44,rv,tv);
    cv::projectPoints(objPoints,rv,tv,imp.CameraMatrix,imp.Distorsion,prepj);
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<prepj.size();i++){
        if ( !std::isnan(objPoints[i].x)){
             sum+= cv::norm( points2d[i]-prepj[i]);
              nvalid++;
        }
    }
    return sum/double(nvalid);
}
```


`marker_m2g`

type: `std::map<uint32_t,cv::Mat>`  

usage: `marker_m2g[3] = T_mg` where `T_mg` is the 4x4 transformation matrix from map to global coordinate and 3 is the id of the marker.

$$p_m = T_{mg} p_g$$


`ml`

type: `minfo`

attribute: `rt_f2m` is the 4x4 transformation matrix from camera to map coordinate


```c++

            //take the all poses and select the one that minimizes the global reproj error
            for(auto & ml:all_marker_locations){

                // rt_f2m is the 4x4 transformation matrix from camera to map coordinate
                auto pose= ml.rt_f2m *marker_m2g[ml.id];
                //now,  compute the repj error of all markers using this info
                ml.err=aruco_private::reprj_error(markerPoints3d,markerPoints2d,_cam_params,  pose);
              }
            //sort and get the best
            std::sort(all_marker_locations.begin(),all_marker_locations.end(),[](const minfo &a,const minfo &b){return a.err<b.err;});
            std::cerr<<"err="<<all_marker_locations.front().err<<std::endl;
            auto &best=all_marker_locations.front();
            pose_f2g_out=best.rt_f2m *marker_m2g[best.id];
        }

        if ( pose_f2g_out.empty()  &&  good_marker_locations.size()>0){
            std::sort(good_marker_locations.begin(),good_marker_locations.end(),[](const minfo &a,const minfo &b){return a.err<b.err;});
            auto best=good_marker_locations[0];
            //estimate current location
            pose_f2g_out= best.rt_f2m *marker_m2g[best.id];
        }
        return   pose_f2g_out;
    }

```





the function `relocalization` will call the following function `solvePnP_`, which will in turn call `solvePoseOfCentredSquare`, `solvePnP_` takes the marker size, image point in 2d ($\omega(u)$) and camera parameters as argument.  

```c++
std::vector<std::pair<cv::Mat,double> > solvePnP_(float size,const  std::vector<cv::Point2f> &imgPoints, cv::InputArray cameraMatrix, cv::InputArray distCoeffs){
    cv::Mat   Rvec, Tvec, Rvec2,Tvec2;
    float reprojErr1, reprojErr2;
    solvePoseOfCentredSquare(size, imgPoints,  cameraMatrix, distCoeffs,   Rvec, Tvec,reprojErr1,Rvec2,Tvec2,reprojErr2);
    return {make_pair(getRTMatrix(Rvec,Tvec,CV_32F),reprojErr1), make_pair(getRTMatrix(Rvec2,Tvec2,CV_32F),reprojErr2) } ;


}
```
