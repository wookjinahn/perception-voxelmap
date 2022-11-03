//
// Created by wj on 22. 4. 19.
//

#include "VoxelmapCore/Point3.hpp"

namespace voxelmapcore
{
    Point3::Point3()
        : camelVector::Point3D()
    {
    }

    Point3::Point3(float x, float y, float z)
        : camelVector::Point3D(x, y, z)
    {
    }

    Point2 Point3::GetNodeKey() const
    {
        return mNodeKey;
    }

    Point2 Point3::GetCentroid() const
    {
        return mCentroid;
    }

    void Point3::SetNodeKey(const Point2& nodeKey)
    {
        mNodeKey = nodeKey;
    }

    void Point3::SetNodeKeyXZ(float x, float z)
    {
        mNodeKey.SetX(x);
        mNodeKey.SetZ(z);
    }

    void Point3::SetCentroid(const Point2& centroid)
    {
        mCentroid = centroid;
    }

//    void Point3::RotationRoll(float degree)
//    {
//        float rotationMatrix[3][3] = {{ 1.0f, 0.0f, 0.0f },
//                                      { 0.0f, (float)std::cos(degree * D2R), (float)-std::sin(degree * D2R) },
//                                      { 0.0f, (float)std::sin(degree * D2R), (float)std::cos(degree * D2R) }};
//
//        float rotatedX = rotationMatrix[0][0] * this->GetX() + rotationMatrix[0][1] * this->GetY() + rotationMatrix[0][2] * this->GetZ();
//        float rotatedY = rotationMatrix[1][0] * this->GetX() + rotationMatrix[1][1] * this->GetY() + rotationMatrix[1][2] * this->GetZ();
//        float rotatedZ = rotationMatrix[2][0] * this->GetX() + rotationMatrix[2][1] * this->GetY() + rotationMatrix[2][2] * this->GetZ();
//        this->SetXYZ(rotatedX, rotatedY, rotatedZ);
//    }

    float Point3::GetDistanceBetween2D(const Point2& other)
    {
        return std::sqrt((GetX() - other.GetX()) * (GetX() - other.GetX()) + (GetZ() - other.GetZ()) * (GetZ() - other.GetZ()));
    }
}