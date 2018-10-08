#ifndef PCDVIEW_H
#define PCDVIEW_H

#include "pcdparse.h"

#include <QString>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#ifndef PCL_TYPEDEFS
#define PCL_TYPEDEFS
typedef pcl::PointCloud<pcl::PointXYZ> XYZCloud;
typedef pcl::PointCloud<pcl::PointXYZI> XYZICloud;
#endif

class PCDView
{
public:
    PCDView();

    void viewCloud(XYZCloud &cloud);
    void viewCloud(XYZICloud &cloud);
    void viewCloud(const QString &pcdFilePath,
                   const bool &intensity = false);
    void viewCloud(const QString &csvFilePath, const QString &delim,
                   const QString &intensityID = "");

private:

};

#endif // PCDVIEW_H
