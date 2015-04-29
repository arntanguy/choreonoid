/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H
#define CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H

#include <cnoid/SceneShape>
#include <boost/optional.hpp>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT SgMesh* createSurfaceMesh(SgPointSet* pointSet);
CNOID_EXPORT boost::optional<double> alignPointCloud(SgPointSet* target, SgPointSet* source, Affine3& io_T, double maxCorrespondenceDistance, int maxIterations);
CNOID_EXPORT boost::optional<double> alignPointCloudPointToPoint(SgPointSet* target, SgPointSet* source, Affine3& io_T, double maxCorrespondenceDistance, int maxIterations, double maxVariation);

}

#endif
