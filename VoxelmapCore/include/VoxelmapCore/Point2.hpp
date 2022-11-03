//
// Created by wj on 22. 4. 19.
//

#ifndef PERCEPTION_VOXELMAP_POINT2_HPP
#define PERCEPTION_VOXELMAP_POINT2_HPP

#include <camel-euclid/Vector.hpp>

namespace voxelmapcore
{

    class Point2 : public camelVector::Point2D
    {
    public:
        Point2();
        Point2(float x, float z);
    };

}

#endif //PERCEPTION_VOXELMAP_POINT2_HPP
