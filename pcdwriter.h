#ifndef PCDBINARYWRITER_H
#define PCDBINARYWRITER_H

#define DEFAULT_INTENSITY 100.0

#include <QDebug>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

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

private:

    bool writeIntensity_;
    bool addFalsePt_; // If true, will add points to cloud even if
                      // intensity is required and it is not provided
                      // or vice versa

    pcl::PointCloud<pcl::PointXYZ> xyzCloud_;
    pcl::PointCloud<pcl::PointXYZI> xyziCloud_;
};

#endif // PCDBINARYWRITER_H
