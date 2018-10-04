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


void PCDWriter::findColIndex(const QString &line, const QString &searched,
                          int &index, const char &delim)
{
    QStringList flList(line.split(delim));
    index = flList.indexOf(searched);
}


bool PCDWriter::parsePointXYZ(const QStringList &lineList, const int &xIndex,
                              const int &yIndex, const int &zIndex)
{
    // Check for indexing errors
    int listSize = lineList.size();
    if ( listSize <= xIndex ||
         listSize <= yIndex ||
         listSize <= zIndex )
        return false;

    bool ok(false);
    float x(0.0), y(0.0), z(0.0);

    // Read x, y and z and check their validity
    x = lineList.at(xIndex).toFloat(&ok);
    if ( !ok )
        return false;

    y = lineList.at(yIndex).toFloat(&ok);
    if ( !ok )
        return false;

    z = lineList.at(zIndex).toFloat(&ok);
    if ( !ok )
        return false;

    addPoint(x,y,z);

    return true;
}


bool PCDWriter::parsePointXYZI(const QStringList &lineList, const int &xIndex,
                               const int &yIndex, const int &zIndex,
                               const int &intensityIndex)
{
    // Check for indexing errors
    int listSize = lineList.size();
    if ( listSize <= xIndex ||
         listSize <= yIndex ||
         listSize <= zIndex )
        return false;

    bool ok(false);
    float x(0.0), y(0.0), z(0.0), intensity(0.0);

    // Read x, y and z and check their validity
    x = lineList.at(xIndex).toFloat(&ok);
    if ( !ok )
        return false;

    y = lineList.at(yIndex).toFloat(&ok);
    if ( !ok )
        return false;

    z = lineList.at(zIndex).toFloat(&ok);
    if ( !ok )
        return false;

    intensity = lineList.at(intensityIndex).toFloat(&ok);
    if ( !ok )
        return false;

    addPoint(x,y,z, intensity);

    return true;
}


bool PCDWriter::convertToPCD(const QString &filePath, const char &delim,
                             const QString &newPath, const FileType &type,
                             const QString intensityId)
{
    // Try to open the file
    QFile originalFile(filePath);
    if ( !originalFile.open(QIODevice::ReadOnly) )
    {
        qDebug() << "ERROR: Could not find given file. File path given: "
                    + filePath;
        return false;
    }

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
    if ( intensityId != "" )
    {
        findColIndex(firstLine, intensityId, intensityIndex, delim);
        writeIntensity_ = true;
    } else
        writeIntensity_ = false;

    if ( intensityId == -1 )
    {
        qDebug() << "ERROR: Could not find intensity column.";
        if ( originalFile.isOpen() )
            originalFile.close();
        return false;
    }

    // Find other column indices
    int xIndex(-1), yIndex(-1), zIndex(-1);
    findColIndex(firstLine, "x", xIndex, delim);
    findColIndex(firstLine, "y", yIndex, delim);
    findColIndex(firstLine, "z", zIndex, delim);

    if ( xIndex == -1 ||
         yIndex == -1 ||
         zIndex == -1 )
    {
        qDebug() << "ERROR: Could not find coordinate columns.";
        if ( originalFile.isOpen() )
            originalFile.close();
        return false;
    }

    // Parse the file and create a new point cloud
    QStringList lineList;

    // Store points without intensity values
    if ( !writeIntensity_ )
    {
        while ( !ts.atEnd() )
        {
            lineList = ts.readLine().split(delim);
            parsePointXYZ(lineList, xIndex, yIndex, zIndex);
        }
    } else
    {
        while ( !ts.atEnd() )
        {
            lineList = ts.readLine().split(delim);
            parsePointXYZI(lineList, xIndex, yIndex, zIndex, intensityIndex);
        }
    }

    if ( originalFile.isOpen() )
        originalFile.close();

    // Create the PCD file
    if ( type == ASCII )
        writeAscii(newPath);
    else if ( type == binary )
        writeBinary(newPath);
    else
    {
        qDebug() << "ERROR: Unknown target file type.";
    }

    return true;
}
