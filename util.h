#ifndef UTIL_H
#define UTIL_H

#include "pcdparse.h"
#include <QString>
#include <pcl/io/pcd_io.h>

void CSVtoPCDfile(const QString &srcPath, const QString &destPath,
                  const QString &delim, const QString &intensityID = "");

void CSVtoBinaryPCDfile(const QString &srcPath, const QString &destPath,
                  const QString &delim, const QString &intensityID = "");

#endif // UTIL_H
