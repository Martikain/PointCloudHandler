#ifndef PCDPARSER_H
#define PCDPARSER_H

#include <QtDebug>
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


bool CSVtoPCD
        (XYZCloud &cloud, QString filePath,
         QString delim);

bool CSVtoPCDwithIntensity
        (XYZICloud &cloud, QString filePath,
        QString delim, QString intensityID);



#endif // PCDPARSER_H
