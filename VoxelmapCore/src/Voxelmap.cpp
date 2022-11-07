//
// Created by canine on 22. 10. 23.
//

#include "VoxelmapCore/Voxelmap.hpp"

namespace voxelmapcore
{
    /**
     * Constructor with BoundingBox and resolution
     * @param dataRange
     * @param resolution
     */
    Voxelmap::Voxelmap(BoundingBox dataRange, float resolution)
        : mResolution(resolution)
    {
        mCellCount = static_cast<int>(dataRange.GetW() / mResolution);

        float dataRangeWidth = mCellCount * mResolution;
        mDataRange = BoundingBox(dataRange.GetX(), dataRange.GetY(), dataRange.GetZ(), dataRangeWidth);

        mCurrentMatrixData = new float** [mCellCount * 2];
        mPastMatrixData = new float** [mCellCount * 2];
        for (int i = 0; i < mCellCount * 2; i++)
        {
            mCurrentMatrixData[i] = new float* [mCellCount * 2];
            mPastMatrixData[i] = new float* [mCellCount * 2];
            for (int j = 0; i < mCellCount * 2; j++)
            {
                mCurrentMatrixData[i][j] = new float[mCellCount * 2];
                mPastMatrixData[i][j] = new float[mCellCount * 2];
            }
        }
    }

    /**
     * Contructor with x, y, width value that consist of BoundingBox and resolution.
     * @param dataRangeX
     * @param dataRangeY
     * @param dataRangeWidth
     * @param resolution
     */
    Voxelmap::Voxelmap(float dataRangeX, float dataRangeY, float dataRangeZ, float dataRangeWidth, float resolution)
        : mResolution(resolution)
    {
        std::cout << "[ Constructor ]" << std::endl;
        mCellCount = static_cast<int>(dataRangeWidth / mResolution);
        std::cout << "mCellCount : " << mCellCount << std::endl;
        dataRangeWidth = mCellCount * resolution;
        std::cout << "dataRangeWidth : " << dataRangeWidth << std::endl;

        mDataRange = BoundingBox(dataRangeX, dataRangeY, dataRangeZ, dataRangeWidth);

        mCurrentMatrixData = new float** [mCellCount * 2];
        mPastMatrixData = new float** [mCellCount * 2];
        for (int i = 0; i < mCellCount * 2; i++)
        {
            mCurrentMatrixData[i] = new float* [mCellCount * 2];
            mPastMatrixData[i] = new float* [mCellCount * 2];
            for (int j = 0; j < mCellCount * 2; j++)
            {
                mCurrentMatrixData[i][j] = new float[mCellCount * 2];
                mPastMatrixData[i][j] = new float[mCellCount * 2];
            }
        }
        std::cout << "[ Constructor End ]" << std::endl;
    }

    Voxelmap::Voxelmap(float dataRangeX, float dataRangeY, float dataRangeZ, float dataRangeWidth, float resolution, float cameraPositionX, float cameraPositionY, float cameraPositionZ)
        : mResolution(resolution)
    {
        std::cout << "[ Constructor ]" << std::endl;
        mCellCount = static_cast<int>(dataRangeWidth / mResolution);

        dataRangeWidth = mCellCount * resolution;
        mDataRange = BoundingBox(dataRangeX, dataRangeY, dataRangeZ, dataRangeWidth);

        mCurrentMatrixData = new float** [mCellCount * 2];
        mPastMatrixData = new float** [mCellCount * 2];
        for (int i = 0; i < mCellCount * 2; i++)
        {
            mCurrentMatrixData[i] = new float* [mCellCount * 2];
            mPastMatrixData[i] = new float* [mCellCount * 2];
            for (int j = 0; j < mCellCount * 2; j++)
            {
                mCurrentMatrixData[i][j] = new float[mCellCount * 2];
                mPastMatrixData[i][j] = new float[mCellCount * 2];
            }
        }

        mCameraPosition[0] = cameraPositionX;
        mCameraPosition[1] = cameraPositionY;
        mCameraPosition[2] = cameraPositionZ;
        std::cout << "[ Constructor End ]" << std::endl;
    }

    Voxelmap::~Voxelmap()
    {
        for (int i = 0; i < mCellCount * 2; i++)
        {
            for (int j = 0; j < mCellCount * 2; j++)
            {
                delete[] mCurrentMatrixData[i][j];
                delete[] mPastMatrixData[i][j];
            }
            delete[] mCurrentMatrixData[i];
            delete[] mPastMatrixData[i];
        }
        delete[] mCurrentMatrixData;
        delete[] mPastMatrixData;
    }

    void Voxelmap::SetDataRage(const BoundingBox& dataRange)
    {
        mDataRange = dataRange;
    }

    void Voxelmap::SetDefaultCameraPosition(float t265PoseZ, float d435PoseX, float d435PoseY, float d435PoseZ)
    {
        mCameraPosition[0] = t265PoseZ;
        mCameraPosition[1] = d435PoseX;
        mCameraPosition[2] = d435PoseY;
        mCameraPosition[3] = d435PoseZ;
    }

    int Voxelmap::GetCellCount() const
    {
        return mCellCount;
    }

    std::vector<Point3> Voxelmap::GetPastData() const
    {
        return mPastData;
    }

    void Voxelmap::SetPastData(const std::vector<Point3>& pastData)
    {
        mPastData = pastData;
    }

    void Voxelmap::SetCurrentOdom(float poseX, float poseY, float poseZ, float quaternionX, float quaternionY, float quaternionZ, float quaternionW)
    {
        mCurrentOdom[0] = poseX;
        mCurrentOdom[1] = poseY;
        mCurrentOdom[2] = poseZ;
        mCurrentOdom[3] = quaternionX;
        mCurrentOdom[4] = quaternionY;
        mCurrentOdom[5] = quaternionZ;
        mCurrentOdom[6] = quaternionW;
    }

    void Voxelmap::FromPCD(const std::string& filePath)
    {
        std::ifstream fin;
        fin.open(filePath);
        std::string line;

        if (fin.is_open())
        {
            int num = 1;
            while (!fin.eof())
            {
                getline(fin, line);
                if (num > 10)
                {
                    float x, y, z;
                    std::istringstream iss(line);
                    iss >> x >> y >> z;

                    Point3 pointXYZ = { x, y, z };
                    mCurrentData.push_back(pointXYZ);
                }
                num++;
            }
        }
        fin.close();
    }

    void Voxelmap::ToPCD(const std::string& filePath)
    {
        time_t t;
        struct tm* timeinfo;
        time(&t);
        timeinfo = localtime(&t);

        std::string hour, min;

        if (timeinfo->tm_hour < 10)
        {
            hour = "0" + std::to_string(timeinfo->tm_hour);
        }
        else
        {
            hour = std::to_string(timeinfo->tm_hour);
        }

        if (timeinfo->tm_min < 10)
        {
            min = "0" + std::to_string(timeinfo->tm_min);
        }
        else
        {
            min = std::to_string(timeinfo->tm_min);
        }

        std::string outputPath = filePath + hour + min + ".pcd";

        std::ofstream fout;
        fout.open(outputPath);

        fout << "VERSION" << std::endl;
        fout << "FIELDS x y z" << std::endl;
        fout << "SIZE 4 4 4" << std::endl;
        fout << "TYPE F F F" << std::endl;
        fout << "COUNT 1 1 1" << std::endl;
        fout << "WIDTH 1" << std::endl;
        fout << "HEIGHT " << mProcessedData.size() << std::endl;
        fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
        fout << "POINTS " << mProcessedData.size() << std::endl;
        fout << "DATA ascii" << std::endl;

        for (int i = 0; i < mProcessedData.size(); i++)
        {
            fout << mProcessedData[i].GetX() << " " << mProcessedData[i].GetY() << " " << mProcessedData[i].GetZ() << "\n";
        }

        fout.close();
    }

    void Voxelmap::FromPointCloud2Msgs(sensor_msgs::PointCloud2 pointcloud2Msgs)
    {
        std::cout << "[ FromPointCloud2Msgs ]" << std::endl;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud2Msgs, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud2Msgs, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud2Msgs, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
            {
                Point3 pointXYZ = { *iter_x, *iter_y, *iter_z };
                mCurrentData.push_back(pointXYZ);
            }
        }
        std::cout << "[ FromPointCloud2Msgs End ] mCurrentData size : " << mCurrentData.size() << "\n" << std::endl;
    }

    void Voxelmap::ToPointCloud2Msgs(const std::string& frame_id, sensor_msgs::PointCloud2& outPointCloud2)
    {
        std::cout << "[ ToPointCloud2Msgs ]" << std::endl;
        sensor_msgs::PointCloud pointcloud_msgs;

        pointcloud_msgs.header.frame_id = frame_id;
        pointcloud_msgs.header.stamp = ros::Time::now();
        pointcloud_msgs.points.resize(mProcessedData.size());

        for (int i = 0; i < pointcloud_msgs.points.size(); i++)
        {
            Point3 outData = mProcessedData[i];
            pointcloud_msgs.points[i].x = outData.GetX();
            pointcloud_msgs.points[i].y = outData.GetY();
            pointcloud_msgs.points[i].z = outData.GetZ();
        }
        sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msgs, outPointCloud2);
        std::cout << "[ ToPointCloud2Msgs End ] mProcessedData size : " << mProcessedData.size() << "\n" << std::endl;
    }

    void Voxelmap::ToVoxelmapMSgs(std::string frame_id, sensor_msgs::PointCloud2& outVoxelmapMsgs)
    {
        std::cout << "[ ToHeightmapMSgs ]" << std::endl;
        sensor_msgs::PointCloud pointcloud_msgs;

        pointcloud_msgs.header.frame_id = frame_id;
        pointcloud_msgs.header.stamp = ros::Time::now();
        pointcloud_msgs.points.resize(mProcessedData.size());

        for (int i = 0; i < pointcloud_msgs.points.size(); i++)
        {
            Point3 outData = mProcessedData[i];
            pointcloud_msgs.points[i].x = outData.GetX();
            pointcloud_msgs.points[i].y = outData.GetY();
            pointcloud_msgs.points[i].z = outData.GetZ();
        }
        sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msgs, outVoxelmapMsgs);
        std::cout << "[ ToHeightmapMSgs End ] mProcessedData size : " << mProcessedData.size() << "\n" << std::endl;
    }

    void Voxelmap::Process()
    {
        std::cout << "[ Process ]" << std::endl;

        changeDataCoordinateTo(mCurrentData, eCoordinate::WORLD);
        initializeMap();    // -10.0f;
        rotateDefaultPosition();
        rotateByOdom(mCurrentData);
        insertDataToEachCell();
        combinePastAndCurrent();
        convertMapToVector();
        SetPastData(mProcessedData);
        std::cout << "[ Process End ]" << "\n" << std::endl;
    }

    /**
     *
     * @param rawData
     * @param dataCoordinate
     */
    void Voxelmap::changeDataCoordinateTo(std::vector<Point3>& data, eCoordinate dataCoordinate)
    {
        std::cout << "[ changeDataCoordinateTo ]" << std::endl;

        switch (dataCoordinate)
        {
        case eCoordinate::WORLD:
            for (auto& iter : data)
            {
                float x = iter.GetZ();
                float y = -(iter.GetX());
                float z = -(iter.GetY());

                iter.SetXYZ(x, y, z);
            }
            break;
        case eCoordinate::CAMERA:
            for (auto& iter : data)
            {
                float x = -(iter.GetY());
                float y = -(iter.GetZ());
                float z = iter.GetX();

                iter.SetXYZ(x, y, z);
            }
            break;
        case eCoordinate::IMU:
            break;
        default:
            std::cout << "!! DEFAULT !!" << std::endl;
            break;
        }
        std::cout << "[ changeDataCoordinateTo End ]" << std::endl;
    }

    void Voxelmap::rotateDefaultPosition()
    {
        std::cout << "[ rotateDefaultPosition ]" << std::endl;
        float defaultRotateAngle = 0.3263766f;

        float pitch00 = cos(defaultRotateAngle);
        float pitch01 = 0.0f;
        float pitch02 = sin(defaultRotateAngle);
        float pitch10 = 0.0f;
        float pitch11 = 1.0f;
        float pitch12 = 0.0f;
        float pitch20 = -sin(defaultRotateAngle);
        float pitch21 = 0.0f;
        float pitch22 = cos(defaultRotateAngle);

        for (int i = 0; i < mCurrentData.size(); i++)
        {
            Point3 data = mCurrentData[i];

            float resultX = pitch00 * data.GetX() + pitch01 * data.GetY() + pitch02 * data.GetZ() + mCameraPosition[1];
            float resultY = pitch10 * data.GetX() + pitch11 * data.GetY() + pitch12 * data.GetZ() + mCameraPosition[2];
            float resultZ = pitch20 * data.GetX() + pitch21 * data.GetY() + pitch22 * data.GetZ() + mCameraPosition[3];

            mCurrentData[i].SetXYZ(resultX, resultY, resultZ);
        }
        std::cout << "[ rotateDefaultPosition End ]" << std::endl;
    }

    void Voxelmap::rotateByOdom(std::vector<Point3>& outData)
    {
        std::cout << "[ rotateByOdom ]" << std::endl;

        float qxx = mCurrentOdom[3] * mCurrentOdom[3];
        float qyy = mCurrentOdom[4] * mCurrentOdom[4];
        float qzz = mCurrentOdom[5] * mCurrentOdom[5];
        float qxz = mCurrentOdom[3] * mCurrentOdom[5];
        float qxy = mCurrentOdom[3] * mCurrentOdom[4];
        float qyz = mCurrentOdom[4] * mCurrentOdom[5];
        float qwx = mCurrentOdom[6] * mCurrentOdom[3];
        float qwy = mCurrentOdom[6] * mCurrentOdom[4];
        float qwz = mCurrentOdom[6] * mCurrentOdom[5];

        float rotate00 = 1 - 2 * (qyy + qzz);
        float rotate01 = 2 * (qxy - qwz);
        float rotate02 = 2 * (qxz + qwy);

        float rotate10 = 2 * (qxy + qwz);
        float rotate11 = 1 - 2 * (qxx + qzz);
        float rotate12 = 2 * (qyz - qwx);

        float rotate20 = 2 * (qxz - qwy);
        float rotate21 = 2 * (qyz + qwx);
        float rotate22 = 1 - 2 * (qxx + qyy);

        for (int i = 0; i < outData.size(); i++)
        {
            Point3 rotatedData = outData[i];

            float resultX = rotate00 * rotatedData.GetX() + rotate01 * rotatedData.GetY() + rotate02 * rotatedData.GetZ() + mCurrentOdom[0];
            float resultY = rotate10 * rotatedData.GetX() + rotate11 * rotatedData.GetY() + rotate12 * rotatedData.GetZ() + mCurrentOdom[1];
            float resultZ = rotate20 * rotatedData.GetX() + rotate21 * rotatedData.GetY() + rotate22 * rotatedData.GetZ() + mCurrentOdom[2];

            outData[i].SetXYZ(resultX, resultY, resultZ);
        }
        std::cout << "[ rotateByOdom End ]" << std::endl;
    }

    void Voxelmap::initializeMap()
    {
        std::cout << "[ initializeMap ]" << std::endl;

        for (int i = 0; i < mCellCount * 2; i++)
        {
            for (int j = 0; j < mCellCount * 2; j++)
            {
                for (int k = 0; k < mCellCount * 2; k++)
                {
                    mCurrentMatrixData[i][j][k] = -100.0f;
                    mPastMatrixData[i][j][k] = -100.0f;
                }
            }
        }
        std::cout << "[ initializeMap End ]" << std::endl;
    }

    void Voxelmap::insertDataToEachCell()
    {
        std::cout << "[ insertDataToEachCell ]" << std::endl;

        for (auto& pastData : mPastData)
        {
            if (mDataRange.IsContained(pastData))
            {
                int xIndex = static_cast<int>(pastData.GetX() / mResolution) + mCellCount - static_cast<int>(mCurrentOdom[0] / mResolution);
                int yIndex = static_cast<int>(pastData.GetY() / mResolution) + mCellCount - static_cast<int>(mCurrentOdom[1] / mResolution);
                int zIndex = static_cast<int>(pastData.GetZ() / mResolution) + mCellCount - static_cast<int>(mCurrentOdom[2] / mResolution);

                if (xIndex >= mCellCount * 2)
                {
                    xIndex = mCellCount * 2 - 1;
                }
                if (yIndex >= mCellCount * 2)
                {
                    yIndex = mCellCount * 2 - 1;
                }
                if (zIndex >= mCellCount * 2)
                {
                    zIndex = mCellCount * 2 - 1;
                }

                mPastMatrixData[xIndex][yIndex][zIndex] = 1;
            }
        }

        for (auto& currentData : mCurrentData)
        {
            if (mDataRange.IsContained(currentData))
            {
                int xIndex = static_cast<int>(currentData.GetX() / mResolution) + mCellCount - static_cast<int>(mCurrentOdom[0] / mResolution);
                int yIndex = static_cast<int>(currentData.GetY() / mResolution) + mCellCount - static_cast<int>(mCurrentOdom[1] / mResolution);
                int zIndex = static_cast<int>(currentData.GetZ() / mResolution) + mCellCount - static_cast<int>(mCurrentOdom[2] / mResolution);

                if (xIndex >= mCellCount * 2)
                {
                    xIndex = mCellCount * 2 - 1;
                }
                if (yIndex >= mCellCount * 2)
                {
                    yIndex = mCellCount * 2 - 1;
                }
                if (zIndex >= mCellCount * 2)
                {
                    zIndex = mCellCount * 2 - 1;
                }
                mCurrentMatrixData[xIndex][yIndex][zIndex] = 1;
            }
        }
        std::cout << "[ insertDataToEachCell End ]" << std::endl;
    }

    void Voxelmap::combinePastAndCurrent()
    {
        for (int i = 0; i < mCellCount * 2; i++)
        {
            for (int j = 0; j < mCellCount * 2; j++)
            {
                for (int k = 0; k < mCellCount * 2; k++)
                {
                    if (mCurrentMatrixData[i][j][k] != -100.0f)
                    {
                        mPastMatrixData[i][j][k] = mCurrentMatrixData[i][j][k];
                    }
                }
            }
        }
    }

    void Voxelmap::convertMapToVector()
    {
        std::cout << "[ convertMapToVector ]" << std::endl;

        for (int i = 0; i < mCellCount * 2; i++)
        {
            for (int j = 0; j < mCellCount * 2; j++)
            {
                for (int k = 0; k < mCellCount * 2; k++)
                {
                    float x = static_cast<float>(i + static_cast<int>(mCurrentOdom[0] / mResolution) - mCellCount) * mResolution;
                    float y = static_cast<float>(j + static_cast<int>(mCurrentOdom[1] / mResolution) - mCellCount) * mResolution;
                    float z = static_cast<float>(k + static_cast<int>(mCurrentOdom[2] / mResolution) - mCellCount) * mResolution;;

                    if (mPastMatrixData[i][j][k] != -100.0f)
                    {
                        Point3 point(x, y, z);
                        mProcessedData.push_back(point);
                    }
                }
            }
        }

        std::cout << "[ convertMapToVector End ]" << std::endl;
    }

    void Voxelmap::visualizeAtToVoxelmapMsgs()
    {
        std::cout << "[ visualizeAtToVoxelmapMsgs ]" << std::endl;

        changeDataCoordinateTo(mProcessedData, eCoordinate::CAMERA);

        std::cout << "[ visualizeAtToVoxelmapMsgs End ]" << "\n" << std::endl;
    }
}