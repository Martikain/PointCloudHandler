#include "pcdwriter.h"


PCDWriter::PCDWriter(const bool &safeWrite) :  writeIntensity_(false),
                                               addFalsePt_(!safeWrite)
{
}


void PCDWriter::storeIntensity(const bool &val)
{
    clearClouds();
    writeIntensity_ = val;
}


void PCDWriter::addPoint(const float &x, const float &y, const float &z)
{
    if ( writeIntensity_ )
    {
        qDebug() << "ERROR: Trying to add XYZ point "
                    "to cloud requesting intensity.";
        if ( addFalsePt_ )
            addPoint(x,y,z,DEFAULT_INTENSITY);
        return;
    }

    pcl::PointXYZ p(x, y, z);
    xyzCloud_.push_back(p);
}


void PCDWriter::addPoint(const float &x, const float &y,
                         const float &z, const float &intensity)
{
    if ( !writeIntensity_ )
    {
        qDebug() << "ERROR: Trying to add XYZI point "
                    "to cloud without intensity values.";
        if ( addFalsePt_ )
            addPoint(x,y,z);
        return;
    }

    pcl::PointXYZI p(intensity);
    p.x = x;
    p.y = y;
    p.z = z;
    xyziCloud_.push_back(p);
}


void PCDWriter::writeBinary(const QString &filePath)
{
    std::string fp = filePath.toLatin1().toStdString();

    if ( writeIntensity_ )
        pcl::io::savePCDFileBinary(fp, xyziCloud_);
    else
        pcl::io::savePCDFileBinary(fp, xyzCloud_);
}


void PCDWriter::writeAscii(const QString &filePath)
{
    std::string fp = filePath.toLatin1().toStdString();

    if ( writeIntensity_ )
        pcl::io::savePCDFileASCII(fp, xyziCloud_);
    else
        pcl::io::savePCDFileASCII(fp, xyzCloud_);
}



void PCDWriter::clearClouds()
{
    xyzCloud_.clear();
    xyziCloud_.clear();
}


bool PCDWriter::convertToPCD(const QString &filePath, const QString &delim,
                             const QString &newPath, const FileType &type,
                             const QString &intensityId)
{
    clearClouds();

    qDebug() << "Starting PCD file conversion...";

    int numOfPointLines(0), successfulLines(0);

    bool parseOK(false);

    if ( intensityId == "" )
    {
        parseOK = parseCloudXYZ(xyzCloud_, numOfPointLines,
                                successfulLines, filePath, delim);
        writeIntensity_ = false;
    } else
    {
        parseOK = parseCloudXYZI(xyziCloud_, numOfPointLines,
                                      successfulLines, filePath, delim,
                                      intensityId);
        writeIntensity_ = true;
    }

    if ( !parseOK )
        return false;

    // Create the PCD file
    if ( type == ASCII )
        writeAscii(newPath);
    else if ( type == binary )
        writeBinary(newPath);
    else
    {
        qDebug() << "ERROR: Unknown target file type.";
        return false;
    }

    qDebug() << "File succesfully converted.";
    qDebug() << "File path: " + newPath;
    qDebug() << "  * Total number of point lines: " << numOfPointLines;
    qDebug() << "  * Points converted: " << successfulLines;
    qDebug() << "  * Succesfull conversion: "
             << static_cast<double>(successfulLines) /
                static_cast<double>(numOfPointLines) * 100
             << " %";

    clearClouds();

    return true;
}
