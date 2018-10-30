#include "pcdparse.h"

// Functions only used by the parser
namespace
{
bool parsePointXYZ
        (XYZCloud &cloud, const QStringList &lineList,
        int xIndex, int yIndex,
        int zIndex);

bool parsePointXYZI
        (XYZICloud &cloud, const QStringList &lineList,
        int xIndex, int yIndex,
        int zIndex, int intensityIndex);

bool coordinatesFromList
        (const QStringList &list, int xIndex,
         int yIndex, int zIndex,
         float &x, float &y, float &z);

void findColumnIndex
        (QString line, QString searched,
         int &index, QString delim);

bool findCoordinateColumns
        (QString firstLine, QString delim,
        int &xCol, int &yCol, int &zCol);

bool openFile(QString filePath, QFile &file);

bool columnError(QFile &file);

bool emptyFileError(QFile &file);

void conversionSuccessText
        (QString srcPath, int convertedPts,
         int numOfLines);
}


bool CSVtoPCD
    (XYZCloud &cloud, QString filePath, QString delim)
{
    QFile originalFile;
    if ( !openFile(filePath, originalFile) )
    {
        return false;
    }

    QTextStream textStream(&originalFile);
    if ( textStream.atEnd() )
    {
        return emptyFileError(originalFile);
    }

    QString firstLine(textStream.readLine());

    // Find coordinate column indices
    int xCol(-1), yCol(-1), zCol(-1);
    bool columnsFound = findCoordinateColumns
            (firstLine, delim, xCol, yCol, zCol);

    if ( !columnsFound )
    {
        return columnError(originalFile);
    }

    // Parse the file and create a new point cloud
    QStringList lineList;
    int numOfLines(0);
    int convertedPts(0);

    // Store points without intensity values
    while ( !textStream.atEnd() )
    {
        lineList = textStream.readLine().split(delim);
        if ( parsePointXYZ(cloud, lineList, xCol, yCol, zCol) )
        {
            convertedPts++;
        }
        numOfLines++;
    }

    conversionSuccessText(filePath, convertedPts, numOfLines);

    if ( originalFile.isOpen() )
    {
        originalFile.close();
    }
    return true;
}


bool CSVtoPCDwithIntensity
    (XYZICloud &cloud, QString filePath,
     QString delim, QString intensityID)
{
    QFile originalFile;
    if ( !openFile(filePath, originalFile) )
    {
        return false;
    }

    QTextStream textStream(&originalFile);
    if ( textStream.atEnd() )
    {
        return emptyFileError(originalFile);
    }

    QString firstLine(textStream.readLine());

    // Search for the intensity column
    int intensityCol(-1);
    findColumnIndex(firstLine, intensityID, intensityCol, delim);

    if ( intensityCol == -1 )
    {
        return columnError(originalFile);
    }

    int xCol(-1), yCol(-1), zCol(-1);
    bool columnsFound = findCoordinateColumns
            (firstLine, delim, xCol, yCol, zCol);

    if ( !columnsFound )
    {
        return columnError(originalFile);
    }

    QStringList lineList;
    int numOfLines(0);
    int convertedPts(0);

    while ( !textStream.atEnd() )
    {
        lineList = textStream.readLine().split(delim);
        if ( parsePointXYZI(cloud, lineList, xCol, yCol, zCol, intensityCol) )
        {
            convertedPts++;
        }
        numOfLines++;
    }

    conversionSuccessText(filePath, convertedPts, numOfLines);

    if ( originalFile.isOpen() )
    {
        originalFile.close();
    }

    return true;
}


namespace
{
bool coordinatesFromList
    (const QStringList &list, int xIndex,
     int yIndex, int zIndex,
     float &x, float &y, float &z)
{
    bool conversionOk(false);

    x = list.at(xIndex).toFloat(&conversionOk);
    if ( !conversionOk )
        return false;

    y = list.at(yIndex).toFloat(&conversionOk);
    if ( !conversionOk )
        return false;

    z = list.at(zIndex).toFloat(&conversionOk);
    if ( !conversionOk )
        return false;

    return true;
}


bool parsePointXYZ
    (XYZCloud &cloud, const QStringList &lineList,
     int xIndex, int yIndex, int zIndex)
{
    // Check for indexing errors
    int listSize = lineList.size();
    if ( listSize <= xIndex ||
         listSize <= yIndex ||
         listSize <= zIndex )
        return false;

    float x(0.0), y(0.0), z(0.0);
    bool coordsFound = coordinatesFromList
            (lineList, xIndex, yIndex, zIndex, x, y, z);
    if ( !coordsFound )
        return false;

    pcl::PointXYZ p(x, y, z);
    cloud.push_back(p);

    return true;
}


bool parsePointXYZI
    (XYZICloud &cloud, const QStringList &lineList,
     int xIndex, int yIndex,
     int zIndex, int intensityIndex)
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

    bool coordsFound = coordinatesFromList
            (lineList, xIndex, yIndex, zIndex, x, y, z);
    if ( !coordsFound )
    {
        return false;
    }

    intensity = lineList.at(intensityIndex).toFloat(&ok);
    if ( !ok )
    {
        return false;
    }

    pcl::PointXYZI p(intensity);
    p.x = x;
    p.y = y;
    p.z = z;

    cloud.push_back(p);

    return true;
}


void conversionSuccessText
    (QString srcPath, int convertedPts,
     int numOfLines)
{
    qInfo() << "CSV file succesfully parsed.";
    qInfo() << "File path: " << srcPath;
    qInfo() << "  * Total number of point lines: " << numOfLines;
    qInfo() << "  * Points converted: " << convertedPts;
    qInfo() << "  * Succesfull conversion: "
             << static_cast<double>(convertedPts) /
                static_cast<double>(numOfLines) * 100
             << " %";
}


void findColumnIndex
    (QString line, QString searched,
     int &index, QString delim)
{
    QStringList flList(line.split(delim));
    index = flList.indexOf(searched);
}


bool findCoordinateColumns
    (QString firstLine, QString delim,
     int &xCol, int &yCol, int &zCol)
{
    findColumnIndex(firstLine, "x", xCol, delim);
    findColumnIndex(firstLine, "y", yCol, delim);
    findColumnIndex(firstLine, "z", zCol, delim);

    // Check that indices have been found
    if ( xCol == -1 ||
         yCol == -1 ||
         zCol == -1 )
    {
        return false;
    }

    return true;
}


bool openFile(QString filePath, QFile &file)
{
    if ( filePath.isEmpty() )
    {
        qCritical() << "ERROR: File path is empty string.";
        return false;
    }

    file.setFileName(filePath);
    if ( !file.open(QIODevice::ReadOnly) )
    {
        qCritical() << "ERROR: Could not find given file."
                    << "File path given:" + filePath;
        return false;
    }

    qInfo() << "Opened file to convert.";
    return true;
}


bool columnError(QFile &file)
{
    qCritical() << "ERROR: Could not find column.";
    if ( file.isOpen() )
    {
        file.close();
    }
    return false;
}


bool emptyFileError(QFile &file)
{
    qCritical() << "ERROR: File is empty.";
    if ( file.isOpen() )
    {
        file.close();
    }

    return false;
}

} // namespace
