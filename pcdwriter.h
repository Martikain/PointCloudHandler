#ifndef PCDBINARYWRITER_H
#define PCDBINARYWRITER_H

#include <list>
#include <QDebug>
#include <QDataStream>
#include <QString>
#include <QFile>
#include <QDir>

struct PointData
{
    float xOrIntensity;
    float y;
    float z;
    float reserved;
};


class PCDWriter
{
public:
    PCDWriter();


    void storeIntensity(const bool &val);
    void addPoint(const float &x, const float &y, const float &z);
    void addPoint(const float &x, const float &y,
                     const float &z, const float &intensity);

    void writeBinary(const QString &filePath);
    void writeAscii(const QString &filePath);

private:

    bool storeIntensity_;
    std::list<PointData> currentPoints_;

    QString createPcdHeader(const bool &binary);
};

#endif // PCDBINARYWRITER_H
