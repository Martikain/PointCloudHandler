#include "pcdparse.h"


bool parseXYZ(const QStringList &list, const int &xIndex, const int &yIndex, const int &zIndex, float &x, float &y, float &z)
{
    bool ok(false);

    x = list.at(xIndex).toFloat(&ok);
    if ( !ok )
        return false;

    y = list.at(yIndex).toFloat(&ok);
    if ( !ok )
        return false;

    z = list.at(zIndex).toFloat(&ok);
    if ( !ok )
        return false;

    return true;
}



bool parsePointXYZ(XYZCloud &cloud, const QStringList &lineList,
                              const int &xIndex, const int &yIndex,
                              const int &zIndex)
{
    // Check for indexing errors
    int listSize = lineList.size();
    if ( listSize <= xIndex ||
         listSize <= yIndex ||
         listSize <= zIndex )
        return false;

    float x(0.0), y(0.0), z(0.0);
    if ( parseXYZ(lineList, xIndex, yIndex, zIndex, x, y, z) == false )
        return false;

    pcl::PointXYZ p(x, y, z);
    cloud.push_back(p);

    return true;
}



bool parsePointXYZI(XYZICloud &cloud, const QStringList &lineList,
                               const int &xIndex, const int &yIndex,
                               const int &zIndex, const int &intensityIndex)
{
    // Check for indexing errors
    int listSize = lineList.size();
    if ( listSize <= xIndex ||
         listSize <= yIndex ||
         listSize <= zIndex ||
         listSize <= intensityIndex )
        return false;

    bool ok(false);
    float x(0.0), y(0.0), z(0.0), intensity(0.0);

    // Read x, y and z and check their validity
    if ( parseXYZ(lineList, xIndex, yIndex, zIndex, x, y, z) == false )
        return false;

    intensity = lineList.at(intensityIndex).toFloat(&ok);
    if ( !ok )
        return false;

    pcl::PointXYZI p(intensity);
    p.x = x;
    p.y = y;
    p.z = z;

    cloud.push_back(p);

    return true;
}


bool parseCloudXYZ(XYZCloud &cloud, int &numOfLines, int &successfulPts, const QString &filePath, const QString &delim)
{
    // Try to open the file
    QFile originalFile(filePath);
    if ( !originalFile.open(QIODevice::ReadOnly) )
    {
        qDebug() << "ERROR: Could not find given file. File path given: "
                    + filePath;
        return false;
    } else
        qDebug() << "Opened file to convert.";

    QTextStream ts(&originalFile);
    if ( ts.atEnd() )
    {
        qDebug() << "ERROR: Empty file";
        if ( originalFile.isOpen() )
            originalFile.close();
        return false;
    }

    QString firstLine(ts.readLine());

    // Find coordinate column indices
    int xIndex(-1), yIndex(-1), zIndex(-1);
    bool ok = findCoordinateIndices(firstLine, delim, xIndex, yIndex, zIndex);

    if ( !ok )
    {
        qDebug() << "ERROR: Could not find coordinate indices in the file.";
        return false;
    }

    // Parse the file and create a new point cloud
    QStringList lineList;
    numOfLines = 0;
    successfulPts = 0;

    // Store points without intensity values
    while ( !ts.atEnd() )
    {
        lineList = ts.readLine().split(delim);
        if ( parsePointXYZ(cloud, lineList, xIndex, yIndex, zIndex) )
            successfulPts++;
        numOfLines++;
    }

    if ( originalFile.isOpen() )
        originalFile.close();

    return true;
}


bool parseCloudXYZI(XYZICloud &cloud, int &numOfLines, int &successfulPts, const QString &filePath, const QString &delim, const QString &intensityID)
{
    // Try to open the file
    QFile originalFile(filePath);
    if ( !originalFile.open(QIODevice::ReadOnly) )
    {
        qDebug() << "ERROR: Could not find given file. File path given: "
                    + filePath;
        return false;
    } else
        qDebug() << "Opened file to convert.";

    QTextStream ts(&originalFile);
    if ( ts.atEnd() )
    {
        qDebug() << "ERROR: Empty file";
        if ( originalFile.isOpen() )
            originalFile.close();
        return false;
    }

    QString firstLine(ts.readLine());

    // Search for the intensity column
    int intensityIndex(-1);
    findColIndex(firstLine, intensityID, intensityIndex, delim);


    if ( intensityIndex == -1 )
    {
        qDebug() << "ERROR: Could not find intensity column.";
        if ( originalFile.isOpen() )
            originalFile.close();
        return false;
    }

    // Find other column indices
    int xIndex(-1), yIndex(-1), zIndex(-1);
    bool ok = findCoordinateIndices(firstLine, delim, xIndex, yIndex, zIndex);

    if ( !ok )
    {
        qDebug() << "ERROR: Could not find coordinate indices in the file.";
        return false;
    }

    // Parse the file and create a new point cloud
    QStringList lineList;
    numOfLines = 0;
    successfulPts = 0;

    while ( !ts.atEnd() )
    {
        lineList = ts.readLine().split(delim);
        if ( parsePointXYZI(cloud, lineList, xIndex,
                            yIndex, zIndex, intensityIndex) )
            successfulPts++;
        numOfLines++;
    }

    if ( originalFile.isOpen() )
        originalFile.close();

    return true;
}


void findColIndex(const QString &line, const QString &searched,
                  int &index, const QString &delim)
{
    QStringList flList(line.split(delim));
    index = flList.indexOf(searched);
}


bool findCoordinateIndices(const QString &firstLine, const QString &delim,
                           int &xIndex, int &yIndex, int &zIndex)
{
    findColIndex(firstLine, "x", xIndex, delim);
    findColIndex(firstLine, "y", yIndex, delim);
    findColIndex(firstLine, "z", zIndex, delim);

    // Check that indices have been found
    if ( xIndex == -1 ||
         yIndex == -1 ||
         zIndex == -1 )
        return false;

    return true;
}
