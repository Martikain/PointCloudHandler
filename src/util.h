#ifndef UTIL_H
#define UTIL_H

#include "pcdparse.h"
#include <QString>
#include <pcl/io/pcd_io.h>

void CSVtoPCDfile(QString srcPath, QString destPath,
                  QString delim, QString intensityID = "");

void CSVtoBinaryPCDfile(QString srcPath, QString destPath,
                  QString delim, QString intensityID = "");

#endif // UTIL_H
