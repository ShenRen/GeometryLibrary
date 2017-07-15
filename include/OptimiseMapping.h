#pragma once
#include "GppDefines.h"
#include "ITriMesh.h"
#include "IPointCloud.h"
#include "TextureImage.h"
#include "Matrix4x4.h"
#include <vector>

namespace GPP
{
    enum FuseImageColorIdType
    {
        FUSE_IMAGECOLORID_SEQUENTIAL = 0,
        FUSE_IMAGECOLORID_OPTIMAL
    };

    class GPP_EXPORT OptimiseMapping
    {
    public:
        OptimiseMapping();
        ~OptimiseMapping();

        // pointColorIds should all be valid: imageIndex >= 0
        static ErrorCode TransferMappingToMesh(const IPointCloud* pointCloud, const std::vector<ImageColorId>& pointColorIds,
            ITriMesh* triMesh, std::vector<ImageColorId>& meshColorIds, Real densityScale = 1.0, bool smoothGeometry = false);


        static ErrorCode InterpolateImageColorIdsOnMesh(const ITriMesh* triMesh, const std::vector<Int>& validFlags, 
            std::vector<ImageColorId>& imageColorIds);


        // pointCloud should be depth data, have normal
        // transform == NULL if transform is identity
        // triMesh's triangle normal should have been calculated
        static ErrorCode ProjectImageColorIdToMesh(const IPointCloud* pointCloud, const Matrix4x4* transform,
            const std::vector<ImageColorId>& pointColorIds, Real pointDensity, const ITriMesh* triMesh, 
            std::vector<std::pair<Int, ImageColorId> >& meshColorIds);


        // cameraDirList: if it is NULL, api will estimate camera direction automatically.
        // cameraDirList, refImageList, refImageInfos are used in FUSE_IMAGECOLORID_OPTIMAL type
        // faceColorIds: -1 means no valid ImageColorId
        // NOTE: camera direction is opposite to point cloud normal
        static ErrorCode FuseImageColorIds(const ITriMesh* triMesh, const std::vector<std::vector<ImageColorId> >& imageColorIdList,
            std::vector<ImageColorId>& imageColorIds, std::vector<Int>& faceColorIds, FuseImageColorIdType fuseType = FUSE_IMAGECOLORID_SEQUENTIAL, 
            const std::vector<Vector3>* cameraDirList = NULL, const std::vector<std::vector<Color4> >* refImageList = NULL, 
            const std::vector<Int>* refImageInfos = NULL);

        
        // imageColorIds shoule be valid, new ImageColorId will push_back into this vector
        // The format of imageColorIds and faceColorIds is the same as them in FuseImageColorIds
        // faceColorIds: a list of the triangle ImageColorId index. The size of it must be a multiple of three.
        //               And for each triple pair, they are indicating the ImageColorId of the triangle.
        //               The ImageColorId data can be got by the IDs from the imageColorIds.
        //               -1 means invalid ImageColorId and will be interpolated
        static ErrorCode InterpolateFaceImageColorIds(const ITriMesh* triMesh, std::vector<ImageColorId>& imageColorIds, 
            std::vector<Int>& faceColorIds);


        static void RasterizeLine(Int startX, Int startY, Int endX, Int endY, std::vector<Int>& line);

    };
}
