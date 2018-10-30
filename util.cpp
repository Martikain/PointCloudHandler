#include "util.h"


template<typename T>
void writePCDinASCII(T cloud, QString path)
{
    pcl::io::savePCDFileASCII(path.toLatin1().toStdString(), cloud);
    qInfo() << "Saved" << cloud.size()
            << "points to" << path
            << "in ASCII format";
}

template<typename T>
void writePCDinBinary(T cloud, QString path)
{
    pcl::io::savePCDFileBinary(path.toLatin1().toStdString(), cloud);
    qInfo() << "Saved" << cloud.size()
             << "points to" << path
             << "in binary format";
}


void CSVtoPCDfile
    (QString srcPath, QString destPath,
     QString delim, QString intensityID)
{
    bool success(false);

    if ( intensityID.isEmpty() )
    {
        XYZCloud cloud;
        success = CSVtoPCD(cloud, srcPath, delim);
        if ( success )
        {
            writePCDinASCII(cloud, destPath);
        }
    } else
    {
        XYZICloud cloud;
        success = CSVtoPCDwithIntensity
                (cloud, srcPath, delim, intensityID);
        if ( success )
        {
            writePCDinASCII(cloud, destPath);
        }
    }

    if ( !success )
    {
        qCritical() << "ERROR: Failed to parse CSV file."
                    << "No PCD file created.";
    }
}


void CSVtoBinaryPCDfile
    (QString srcPath, QString destPath,
     QString delim, QString intensityID)
{
    bool success(false);

    if ( intensityID.isEmpty() )
    {
        XYZCloud cloud;
        success = CSVtoPCD(cloud, srcPath, delim);
        if ( success )
        {
            writePCDinBinary(cloud, destPath);
        }
    } else
    {
        XYZICloud cloud;
        success = CSVtoPCDwithIntensity
                (cloud, srcPath, delim, intensityID);
        if ( success )
        {
            writePCDinBinary(cloud, destPath);
        }
    }

    if ( !success )
    {
        qCritical() << "ERROR: Failed to parse CSV file."
                    << "No PCD file created.";
    }
}
