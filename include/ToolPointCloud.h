/*==================================================================================================

                       Copyright (c) 2016 GeometryPlusPlus, ThreePark
                             Unpublished - All rights reserved

====================================================================================================*/
#pragma once
#include "PointCloud.h"
#include "IPointList.h"
#include "Matrix4x4.h"
#include <vector>

namespace GPP
{
    extern GPP_EXPORT PointCloud* CopyPointCloud(const PointCloud* pointCloud);

    // deleteIndex could have duplicate index
    extern GPP_EXPORT ErrorCode DeletePointCloudElements(IPointCloud* pointCloud, const std::vector<Int>& deleteIndex);

    extern GPP_EXPORT ErrorCode CalculatePointListBoundingBox(const IPointList* pointlist, Vector3& bboxMin, Vector3& bboxMax);

    extern GPP_EXPORT ErrorCode CalculatePointCloudListBoundingBox(const std::vector<IPointCloud*> pointCloudList, 
        const std::vector<Matrix4x4>* transformList, Vector3& bboxMin, Vector3& bboxMax);

    extern GPP_EXPORT ErrorCode CalculatePointListDensity(const IPointList* pointList, Int neighborCount, Real& density);

    // pointCloud should have normal, and this function just update normal and the orientation is the same as before
    extern GPP_EXPORT ErrorCode UpdatePointCloudNormal(IPointCloud* pointCloud, Int neighborCount);

    // pointCloud should have normal
    // pointCloud should be depth data
    // maxZAngle: (0, 90 * ONE_RADIAN). Remove points whose normal z(positive) in cameral coordinate is less than cos(maxZAngle)
    extern GPP_EXPORT ErrorCode RemoveBoundaryPoint(IPointCloud* pointCloud, Real maxZAngle);

}
