#include "pcdbinarywriter.h"

PCDBinaryWriter::PCDBinaryWriter()
{
}

void PCDBinaryWriter::storeIntensity(const bool &val)
{
    intensity_ = val;
}


void PCDBinaryWriter::binaryWrite(const float &x, const float &y,
                                  const float &z)
{
    PointData newPoint{x, y, z, 0.0};
    currentPoints_.push_back(newPoint);
}


void PCDBinaryWriter::binaryWrite(const float &x, const float &y,
                                  const float &z, const float &intensity)
{
    if ( !intensity_ )
        qDebug() << "WARNING: Intensity value is not stored.";

    binaryWrite(x, y, z);
    PointData intensityVal{intensity, 0.0, 0.0, 0.0};
    currentPoints_.push_back(intensityVal);
}
