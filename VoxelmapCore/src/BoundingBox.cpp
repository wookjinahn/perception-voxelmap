//
// Created by wj on 22. 4. 19.
//

#include "VoxelmapCore/BoundingBox.hpp"

namespace voxelmapcore
{
    BoundingBox::BoundingBox()
        : mX(0.0f)
        , mY(0.0f)
        , mW(0.0f)
        , mH(0.0f)
    {
    }

    BoundingBox::BoundingBox(float x, float y, float w, float h)
        : mX(x)
        , mY(y)
        , mW(w)
        , mH(h)
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

    float BoundingBox::GetW() const
    {
        return mW;
    }

    float BoundingBox::GetH() const
    {
        return mH;
    }

    void BoundingBox::SetBoundary(float x, float y, float w, float h)
    {
        mX = x;
        mY = y;
        mW = w;
        mH = h;
    }

    BoundingBox BoundingBox::GetBoundary() const
    {
        return BoundingBox(mX, mY, mW, mH);
    }

    bool BoundingBox::IsContained(const Point3& point) const
    {
        return (point.GetX() >= mX - mW && point.GetX() < mX + mW && point.GetY() >= mY - mH && point.GetY() < mY + mH);
    }
}