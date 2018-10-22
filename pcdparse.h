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
// return the cloud. If parsing fails, returns false.
bool CSVtoPCD
        (XYZCloud &cloud, int &numOfLines, int &successfulPts,
        const QString &filePath, const QString &delim);

// Sets numOfLines and succesfulPts (number of successfully
// converted points)
bool CSVtoPCDwithIntensity
        (XYZICloud &cloud, int &numOfLines, int &successfulPts,
        const QString &filePath, const QString &delim,
        const QString &intensityID);

bool parsePointXYZ
        (XYZCloud &cloud, const QStringList &lineList,
        const int &xIndex, const int &yIndex,
        const int &zIndex);

bool parsePointXYZI
        (XYZICloud &cloud, const QStringList &lineList,
        const int &xIndex, const int &yIndex,
        const int &zIndex, const int &intensityIndex);

bool coordinatesFromList
        (const QStringList &list, const int &xIndex,
         const int &yIndex, const int &zIndex,
         float &x, float &y, float &z);

void findColumnIndex
        (const QString &line, const QString &searched,
         int &index, const QString &delim);

bool findCoordinateColumns
        (const QString &firstLine, const QString &delim,
        int &xCol, int &yCol, int &zCol);

bool openFile(const QString &filePath, QFile &file);

bool columnError(QFile &file);

bool emptyFileError(QFile &file);

#endif // PCDPARSER_H
