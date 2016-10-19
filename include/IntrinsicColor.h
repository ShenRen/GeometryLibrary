/*==================================================================================================

                       Copyright (c) 2016 GeometryPlusPlus, ThreePark
                             Unpublished - All rights reserved

====================================================================================================*/
#pragma once
#include "IPointCloud.h"
#include "ITriMesh.h"
#include "Matrix4x4.h"
#include <vector>

namespace GPP
{
    enum PixelType
    {
        PIXEL_TYPE_BACKGROUND = 0,
        PIXEL_TYPE_EXTEND,
        PIXEL_TYPE_VERTEX_COLOR,
        PIXEL_TYPE_IMAGE_COLOR
    };

    class GPP_EXPORT IntrinsicColor
    {
    public:
        IntrinsicColor();
        ~IntrinsicColor();

        // Overview: pointColors will be merged into one consistent region by diffusing boundaries of different colorIds
        // pointCloud: it should be uniform. If not, please use TuneMeshColorFromMultiPatch instead
        // neighborCount: range >= 4, usually be 12, larger value will make color fusing more
        // colorIds: points having the same colorId should have consistent color
        // pointColors: range[0, 1]
        static ErrorCode TuneColorFromMultiFrame(const IPointCloud* pointCloud, Int neighborCount, 
            const std::vector<Int>& colorIds, std::vector<Vector3>& pointColors);

        
        // Point cloud should have been aligned before calling this api
        // color value should be in range [0, 1]
        // needBlend: colors will be blended near the boundaries of different colorId
        // pointDensity: if it is NULL, it will be calculated automatically. If pointClouds are not uniformed, please set it manually
        // originColorIdList: points having the same colorId should have consistent color. If it is NULL, points from the same cloud will have the same colorId
        // tunedColorIdList: output point colorIds. points having the same colorId should have consistent color
        static ErrorCode TuneColorFromSingleLight(const std::vector<IPointCloud*>& pointCloudList, 
            std::vector<std::vector<Vector3> >& colorList, bool needBlend, const Real* pointDensity,
            const std::vector<std::vector<Int> >* originColorIdList, std::vector<std::vector<Int> >* tunedColorIdList);


        // Overview: texture image color will be tuned into one consistent region by constrained vertex color
        // textureCoords: constrained vertex texture coordinates: coordX_0, coordY_0, coordX_1, coordY_1...... range: [0, 1]
        // vertexColors: constrained vertex colors
        static ErrorCode TuneTextureImageByVertexColor(const std::vector<Real>& textureCoords, const std::vector<Vector3>& vertexColors, 
            Int imageWidth, Int imageHeight, const std::vector<PixelType> pixelTypes, std::vector<Vector3>& textureColors);


        // colorIds: vertices having the same colorId should have consistent color
        // vertexColors: range[0, 1]
        static ErrorCode TuneMeshColorFromMultiPatch(const ITriMesh* triMesh, const std::vector<Int>& colorIds, 
            std::vector<Vector3>& vertexColors);
    };
}
