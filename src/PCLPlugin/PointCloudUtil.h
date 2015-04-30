/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H
#define CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H

#include <cnoid/SceneShape>
#include <boost/optional.hpp>
#include "exportdecl.h"
#include <icp/result.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cnoid {

CNOID_EXPORT void computePointCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
CNOID_EXPORT SgMesh* createSurfaceMesh(SgPointSet* pointSet);
CNOID_EXPORT boost::optional<double> alignPointCloud(SgPointSet* target, SgPointSet* source, Affine3& io_T, double maxCorrespondenceDistance, int maxIterations);
CNOID_EXPORT icp::IcpResults alignPointCloudPointToPoint(SgPointSet* target, SgPointSet* source, Affine3& io_T, double maxCorrespondenceDistance, int maxIterations, double maxVariation);
CNOID_EXPORT icp::IcpResults alignPointCloudPointToPlane (SgPointSet *target, SgPointSet *source, Affine3 &io_T, double maxCorrespondenceDistance, int maxIterations, double maxVariation);
CNOID_EXPORT icp::IcpResults alignPointCloudPointToPointSim3(SgPointSet* target, SgPointSet* source, Affine3& io_T, double maxCorrespondenceDistance, int maxIterations, double maxVariation);

}

#endif
