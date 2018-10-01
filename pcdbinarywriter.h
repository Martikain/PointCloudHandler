#ifndef PCDBINARYWRITER_H
#define PCDBINARYWRITER_H

#include <list>
#include <Eigen/Core>

union PointXYZ
{
  float data[4];
  struct StructXYZ
  {
    float x;
    float y;
    float z;
  };
};

class PCDBinaryWriter
{
public:
    PCDBinaryWriter();

    void binaryWrite();
};

#endif // PCDBINARYWRITER_H
