#ifndef PCDBINARYWRITER_H
#define PCDBINARYWRITER_H

#include <list>
#include <QDebug>

struct PointData
{
    float xOrIntensity;
    float y;
    float z;
    float reserved;
};


class PCDBinaryWriter
{
public:
    PCDBinaryWriter();


    void storeIntensity(const bool &val);
    void binaryWrite(const float &x, const float &y, const float &z);
    void binaryWrite(const float &x, const float &y,
                     const float &z, const float &intensity);

    bool intensity_;
    std::list<PointData> currentPoints_;

};

#endif // PCDBINARYWRITER_H
