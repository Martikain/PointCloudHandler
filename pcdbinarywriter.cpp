#include "pcdbinarywriter.h"

PCDBinaryWriter::PCDBinaryWriter()
{
}

void PCDBinaryWriter::storeIntensity(const bool &val)
{
    storeIntensity_ = val;
}


void PCDBinaryWriter::addPoint(const float &x, const float &y,
                                  const float &z)
{
    PointData newPoint{x, y, z, 0.0};
    currentPoints_.push_back(newPoint);
}


void PCDBinaryWriter::addPoint(const float &x, const float &y,
                                  const float &z, const float &intensity)
{
    if ( !storeIntensity_ )
        qDebug() << "WARNING: Intensity value is not stored.";

    addPoint(x, y, z);
    PointData intensityVal{intensity, 0.0, 0.0, 0.0};
    currentPoints_.push_back(intensityVal);
}


void PCDBinaryWriter::writeBinary(const QString &filePath)
{

}


void PCDBinaryWriter::writeAscii(const QString &filePath)
{

}


QString PCDBinaryWriter::createPcdHeader(const bool &binary)
{
    QString header = "# .PCD v.7 - Point Cloud Data file format\n";
    header += "VERSION .7\n";
    QString numOfPoints;

    if ( storeIntensity_ )
    {
        numOfPoints = (QString::number(currentPoints_.size()/2));
        header += "FIELDS x y z intensity\n";
        header += "SIZE 4 4 4 4\n";
        header += "TYPE 4 4 4 4\n";
        header += "COUNT 1 1 1 1\n";
    } else
    {
        numOfPoints = (QString::number(currentPoints_.size()));
        header += "FIELDS x y z\n";
        header += "SIZE 4 4 4\n";
        header += "TYPE 4 4 4\n";
        header += "COUNT 1 1 1\n";
    }

    header += "WIDTH " + numOfPoints + "\n";
    header += "HEIGHT 1\n";
    header += "WIEWPOINT 0 0 0 1 0 0 0\n";
    header += "POINTS " + numOfPoints + "\n";

    if ( binary )
        header += "DATA binary\n";
    else
        header += "DATA ascii\n";

    return header;
}
