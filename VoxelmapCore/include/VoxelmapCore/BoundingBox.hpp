//
// Created by wj on 22. 4. 19.
//

#ifndef PERCEPTION_VOXELMAP_BOUNDINGBOX_HPP
#define PERCEPTION_VOXELMAP_BOUNDINGBOX_HPP

#include "Point3.hpp"

namespace voxelmapcore
{
    class BoundingBox
    {
    public:
        BoundingBox();
        BoundingBox(float x, float y, float w, float h);
        float GetX() const;
        float GetY() const;
        float GetW() const;
        float GetH() const;
        void SetBoundary(float x, float y, float w, float h);
        BoundingBox GetBoundary() const;
        bool IsContained(const Point3& point) const;

    private:
        float mX;
        float mY;
        float mW;
        float mH;
    };
}

#endif //PERCEPTION_VOXELMAP_BOUNDINGBOX_HPP
