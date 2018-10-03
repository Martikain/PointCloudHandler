#include "pcdwriter.h"


PCDWriter::PCDWriter(const bool &safeWrite) : addFalsePt_(!safeWrite)
{
}


void PCDWriter::storeIntensity(const bool &val)
{
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
    int i = 0;
}


void PCDWriter::writeAscii(const QString &filePath)
{

}
