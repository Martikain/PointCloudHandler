#ifndef PCDBINARYWRITER_H
#define PCDBINARYWRITER_H

#define DEFAULT_INTENSITY 100.0

#include <QDebug>
#include <string>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

enum FileType{ASCII = 0, binary = 1};

class PCDWriter
{
public:

    // Setting safeWrite to false enables the use of
    // addPoint() function with intensity even if it
    // is not stored or vice versa. If true, calling
    // addPoint() in wrong manner will only cause error
    // texts and no point is added.
    PCDWriter(const bool &safeWrite = true);

    // Defines whether intensity values are stored in the PCD file
    // If true, all future addPoint() calls must have intensity values
    // If false, all future addPoint() calls must omit intensity
    void storeIntensity(const bool &val);

    // Functions for adding points to the point cloud
    void addPoint(const float &x, const float &y, const float &z);
    void addPoint(const float &x, const float &y,
                     const float &z, const float &intensity);

    // Creates a binary format PCD file
    void writeBinary(const QString &filePath);

    // Creates an ASCII format PCD file
    void writeAscii(const QString &filePath);

    // Converts a delimited file into PCD format.
    // If intensityId is given, this will search for intensity values
    // from the column with the first row value intensityId
    bool convertToPCD(const QString &filePath, const QString &delim,
                      const QString &newPath, const FileType &type,
                      const QString intensityId = "");

private:

    bool writeIntensity_;
    bool addFalsePt_; // If true, will add points to cloud even if
                      // intensity is required and it is not provided
                      // or vice versa

    pcl::PointCloud<pcl::PointXYZ> xyzCloud_;
    pcl::PointCloud<pcl::PointXYZI> xyziCloud_;
    
    // Finds the column index of the searched string.
    // Returns -1 if not found.
    // Used for finding x, y, z and intensity columns.
    void findColIndex(const QString &line, const QString &searched,
                   int &index, const QString &delim);

    bool parsePointXYZ(const QStringList &lineList, const int &xIndex,
                       const int &yIndex, const int &zIndex);
    bool parsePointXYZI(const QStringList &lineList, const int &xIndex,
                        const int &yIndex, const int &zIndex, const int &intensityIndex);
};

#endif // PCDBINARYWRITER_H
