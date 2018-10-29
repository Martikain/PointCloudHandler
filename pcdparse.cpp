#include "pcdparse.h"

// Functions only used by the parser
namespace
{
bool parsePointXYZ
        (XYZCloud &cloud, const QStringList &lineList,
        const int &xIndex, const int &yIndex,
        const int &zIndex);

bool parsePointXYZI
        (XYZICloud &cloud, const QStringList &lineList,
        const int &xIndex, const int &yIndex,
        const int &zIndex, const int &intensityIndex);

bool coordinatesFromList
        (const QStringList &list, const int &xIndex,
         const int &yIndex, const int &zIndex,
         float &x, float &y, float &z);

void findColumnIndex
        (const QString &line, const QString &searched,
         int &index, const QString &delim);

bool findCoordinateColumns
        (const QString &firstLine, const QString &delim,
        int &xCol, int &yCol, int &zCol);

bool openFile(const QString &filePath, QFile &file);

bool columnError(QFile &file);

bool emptyFileError(QFile &file);

void conversionSuccessText
        (const QString &srcPath, const int &convertedPts,
         const int &numOfLines);
}


bool CSVtoPCD
    (XYZCloud &cloud, const QString &filePath,
     const QString &delim)
{
    QFile originalFile;
    if ( !openFile(filePath, originalFile) )
        return false;

    QTextStream textStream(&originalFile);
    if ( textStream.atEnd() )
        return emptyFileError(originalFile);

    QString firstLine(textStream.readLine());

    // Find coordinate column indices
    int xCol(-1), yCol(-1), zCol(-1);
    bool columnsFound = findCoordinateColumns
            (firstLine, delim, xCol, yCol, zCol);

    if ( !columnsFound )
        return columnError(originalFile);

    // Parse the file and create a new point cloud
    QStringList lineList;
    int numOfLines(0);
    int convertedPts(0);

    // Store points without intensity values
    while ( !textStream.atEnd() )
    {
        lineList = textStream.readLine().split(delim);
        if ( parsePointXYZ(cloud, lineList, xCol, yCol, zCol) )
            convertedPts++;
        numOfLines++;
    }

    conversionSuccessText(filePath, convertedPts, numOfLines);

    if ( originalFile.isOpen() )
        originalFile.close();

    return true;
}


bool CSVtoPCDwithIntensity
    (XYZICloud &cloud, const QString &filePath,
     const QString &delim, const QString &intensityID)
{
    QFile originalFile;
    if ( !openFile(filePath, originalFile) )
        return false;

    QTextStream textStream(&originalFile);
    if ( textStream.atEnd() )
        return emptyFileError(originalFile);

    QString firstLine(textStream.readLine());

    // Search for the intensity column
    int intensityCol(-1);
    findColumnIndex(firstLine, intensityID, intensityCol, delim);

    if ( intensityCol == -1 )
        return columnError(originalFile);

    int xCol(-1), yCol(-1), zCol(-1);
    bool columnsFound = findCoordinateColumns
            (firstLine, delim, xCol, yCol, zCol);

    if ( !columnsFound )
        return columnError(originalFile);

    QStringList lineList;
    int numOfLines(0);
    int convertedPts(0);

    while ( !textStream.atEnd() )
    {
        lineList = textStream.readLine().split(delim);
        if ( parsePointXYZI(cloud, lineList, xCol, yCol, zCol, intensityCol) )
            convertedPts++;
        numOfLines++;
    }

    conversionSuccessText(filePath, convertedPts, numOfLines);

    if ( originalFile.isOpen() )
        originalFile.close();

    return true;
}


namespace
{
bool coordinatesFromList
    (const QStringList &list, const int &xIndex,
     const int &yIndex, const int &zIndex,
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

    bool coordsFound = coordinatesFromList
            (lineList, xIndex, yIndex, zIndex, x, y, z);
    if ( !coordsFound )
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


void conversionSuccessText
    (const QString &srcPath, const int &convertedPts,
     const int &numOfLines)
{
    qDebug() << "CSV file succesfully parsed.";
    qDebug() << "File path: " << srcPath;
    qDebug() << "  * Total number of point lines: " << numOfLines;
    qDebug() << "  * Points converted: " << convertedPts;
    qDebug() << "  * Succesfull conversion: "
             << static_cast<double>(convertedPts) /
                static_cast<double>(numOfLines) * 100
             << " %";
}


void findColumnIndex
    (const QString &line, const QString &searched,
     int &index, const QString &delim)
{
    QStringList flList(line.split(delim));
    index = flList.indexOf(searched);
}


bool findCoordinateColumns
    (const QString &firstLine, const QString &delim,
     int &xCol, int &yCol, int &zCol)
{
    findColumnIndex(firstLine, "x", xCol, delim);
    findColumnIndex(firstLine, "y", yCol, delim);
    findColumnIndex(firstLine, "z", zCol, delim);

    // Check that indices have been found
    if ( xCol == -1 ||
         yCol == -1 ||
         zCol == -1 )
        return false;

    return true;
}


bool openFile(const QString &filePath, QFile &file)
{
    if ( filePath.isEmpty() )
    {
        qDebug() << "ERROR: File path is empty string.";
        return false;
    }

    file.setFileName(filePath);
    if ( !file.open(QIODevice::ReadOnly) )
    {
        qDebug() << "ERROR: Could not find given file. "
                    "File path given: " + filePath;
        return false;
    } else
    {
        qDebug() << "Opened file to convert.";
        return true;
    }
}


bool columnError(QFile &file)
{
    qDebug() << "ERROR: Could not find column.";
    if ( file.isOpen() )
        file.close();
    return false;
}


bool emptyFileError(QFile &file)
{
    qDebug() << "ERROR: File is empty.";
    if ( file.isOpen() )
        file.close();
    return false;
}

} // namespace
