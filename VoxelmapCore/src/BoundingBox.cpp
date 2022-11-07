//
// Created by wj on 22. 4. 19.
//

#include "VoxelmapCore/BoundingBox.hpp"

namespace voxelmapcore
{
    BoundingBox::BoundingBox()
        : mX(0.0f)
        , mY(0.0f)
        , mZ(0.0f)
        , mW(0.0f)
    {
    }

    BoundingBox::BoundingBox(float x, float y, float z, float w)
        : mX(x)
        , mY(y)
        , mZ(z)
        , mW(w)
    {
    }

    float BoundingBox::GetX() const
    {
        return mX;
    }

    float BoundingBox::GetY() const
    {
        return mY;
    }

    float BoundingBox::GetZ() const
    {
        return mZ;
    }

    float BoundingBox::GetW() const
    {
        return mW;
    }

    void BoundingBox::SetBoundary(float x, float y, float z, float w)
    {
        mX = x;
        mY = y;
        mZ = z;
        mW = w;
    }

    BoundingBox BoundingBox::GetBoundary() const
    {
        return BoundingBox(mX, mY, mZ, mW);
    }

    bool BoundingBox::IsContained(const Point3& point) const
    {
        return (point.GetX() >= mX - mW && point.GetX() < mX + mW && point.GetY() >= mY - mW && point.GetY() < mY + mW && point.GetZ() >= mZ - mW && point.GetZ() < mZ + mW);
    }
}