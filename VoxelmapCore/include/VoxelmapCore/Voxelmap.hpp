//
// Created by canine on 22. 10. 23.
//

#ifndef PERCEPTION_VOXELMAP_VOXELMAP_HPP
#define PERCEPTION_VOXELMAP_VOXELMAP_HPP

#include <fstream>        // file oi (std::ifstream)
#include <iostream>
#include <random>        // sampling
#include <sstream>        // file oi (std::istringstream)
#include <string>        // data path
#include <memory>    // unique_ptr

// STL
#include <algorithm>
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// voxelmap
#include "Point2.hpp"
#include "Point3.hpp"
#include "BoundingBox.hpp"
#include "eCoordinate.hpp"

namespace voxelmapcore
{
    class Voxelmap
    {
    public:
        Voxelmap(BoundingBox dataRange, float resolution);
        Voxelmap(float dataRangeX, float dataRangeY, float dataRangeWidth, float resolution);
        Voxelmap(float dataRangeX, float dataRangeY, float dataRangeWidth, float resolution, float cameraPositionX, float cameraPositionY, float cameraPositionZ);
        ~Voxelmap();

        void SetDataRage(const BoundingBox& dataRange);
        void SetDefaultCameraPosition(float t265PoseZ, float d435PoseX, float d435PoseY, float d435PoseZ);
        int GetCellCount() const;
        std::vector<Point3> GetPastData() const;
        void SetPastData(const std::vector<Point3>& pastData);
        void SetCurrentOdom(float poseX, float poseY, float poseZ, float quaternionX, float quaternionY, float quaternionZ, float quaternionW);

        void FromPCD(const std::string& filePath);
        void ToPCD(const std::string& filePath);
        void FromPointCloud2Msgs(sensor_msgs::PointCloud2 pointcloud2Msgs);
        void ToPointCloud2Msgs(const std::string& frame_id, sensor_msgs::PointCloud2& outPointCloud2);
        void ToHeightmapMSgs(std::string frame_id, HeightmapMsgs::Heightmap& outHeightmapMsgs);

        void Process();
    private:
        static void changeDataCoordinateTo(std::vector<Point3>& data, eCoordinate dataCoordinate);
        void rotateDefaultPosition();
        void rotateByOdom(std::vector<Point3>& outData);
        void initializeMap();
        void insertDataToEachCell();
        void combinePastAndCurrent();
        void convertMapToVector();
        void visualizeAtToHeightmapMsgs();

    private:
        std::vector<Point3> mCurrentData;
        std::vector<Point3> mProcessedData;
        std::vector<Point3> mPastData;

        float mCurrentOdom[7];

        float** mCurrentMatrixData;
        float** mPastMatrixData;

        float mCameraPosition[4];
        BoundingBox mDataRange;
        float mResolution;
        int mCellCount;
    };
}

#endif //PERCEPTION_VOXELMAP_VOXELMAP_HPP
