#ifndef PCDPARSER_H
#define PCDPARSER_H

#include <QDebug>
#include <string>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#ifndef PCL_TYPEDEFS
#define PCL_TYPEDEFS
typedef pcl::PointCloud<pcl::PointXYZ> XYZCloud;
typedef pcl::PointCloud<pcl::PointXYZI> XYZICloud;
#endif

// These functions parse point clouds from CSV files and
// return the cloud. If parsing fails, returns empty cloud
bool parseCloudXYZ(XYZCloud &cloud, int &numOfLines, int &successfulPts,
                   const QString &filePath, const QString &delim);

bool parseCloudXYZI(XYZICloud &cloud, int &numOfLines, int &successfulPts,
                    const QString &filePath, const QString &delim,
                    const QString &intensityID);

bool parsePointXYZ(XYZCloud &cloud, const QStringList &lineList,
                   const int &xIndex, const int &yIndex,
                   const int &zIndex);

bool parsePointXYZI(XYZICloud &cloud, const QStringList &lineList,
                    const int &xIndex, const int &yIndex,
                    const int &zIndex, const int &intensityIndex);

bool parseXYZ(const QStringList &list, const int &xIndex,
              const int &yIndex, const int &zIndex,
              float &x, float &y, float &z);

void findColIndex(const QString &line, const QString &searched,
                  int &index, const QString &delim);

bool findCoordinateIndices(const QString &firstLine, const QString &delim,
                           int &xIndex, int &yIndex, int &zIndex);

#endif // PCDPARSER_H
