#include "util.h"


template<typename T>
void writePCDinASCII(T cloud, const QString &path)
{
    pcl::io::savePCDFileASCII(path.toLatin1().toStdString(), cloud);
    qDebug() << "Saved" << cloud.size()
             << "points to" << path
             << "in ASCII format";
}

template<typename T>
void writePCDinBinary(T cloud, const QString &path)
{
    pcl::io::savePCDFileBinary(path.toLatin1().toStdString(), cloud);
    qDebug() << "Saved" << cloud.size()
             << "points to" << path
             << "in binary format";
}


void CSVtoPCDfile
    (const QString &srcPath, const QString &destPath,
     const QString &delim, const QString &intensityID)
{
    bool success(false);

    if ( intensityID.isEmpty() )
    {
        XYZCloud cloud;
        success = CSVtoPCD(cloud, srcPath, delim);
        if ( success )
            writePCDinASCII(cloud, destPath);
    } else
    {
        XYZICloud cloud;
        success = CSVtoPCDwithIntensity
                (cloud, srcPath, delim, intensityID);
        if ( success )
            writePCDinASCII(cloud, destPath);
    }

    if ( !success )
        qDebug() << "ERROR: Failed to parse CSV file."
                    "No PCD file created.";
}

void CSVtoBinaryPCDfile
    (const QString &srcPath, const QString &destPath,
     const QString &delim, const QString &intensityID)
{
    bool success(false);

    if ( intensityID.isEmpty() )
    {
        XYZCloud cloud;
        success = CSVtoPCD(cloud, srcPath, delim);
        if ( success )
            writePCDinBinary(cloud, destPath);
    } else
    {
        XYZICloud cloud;
        success = CSVtoPCDwithIntensity
                (cloud, srcPath, delim, intensityID);
        if ( success )
            writePCDinBinary(cloud, destPath);
    }

    if ( !success )
        qDebug() << "ERROR: Failed to parse CSV file."
                    "No PCD file created.";
}
