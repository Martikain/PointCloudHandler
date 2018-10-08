#include "pcdview.h"

PCDView::PCDView()
{

}

void PCDView::viewCloud(XYZCloud &cloud)
{
    pcl::visualization::CloudViewer viewer("Cloud visualization");
    XYZCloud::Ptr cloudPtr(new XYZCloud);
    *cloudPtr = cloud;
    viewer.showCloud(cloudPtr);

    while ( !viewer.wasStopped() ){}
}


void PCDView::viewCloud(XYZICloud &cloud)
{
    pcl::visualization::CloudViewer viewer("Cloud visualization");
    XYZICloud::Ptr cloudPtr(new XYZICloud);
    *cloudPtr = cloud;
    viewer.showCloud(cloudPtr);
    while ( !viewer.wasStopped() ){}
}


void PCDView::viewCloud(const QString &pcdFilePath, const bool &intensity)
{
    const char *filePathC = pcdFilePath.toLatin1().toStdString().c_str();

    if ( intensity )
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud
                (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile (filePathC, *cloud);

        viewCloud(*cloud);
    } else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
                (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile (filePathC, *cloud);

        viewCloud(*cloud);
    }
}


void PCDView::viewCloud(const QString &csvFilePath, const QString &delim,
                        const QString &intensityID)
{
    int lines, success;

    if ( intensityID == "" )
    {
        XYZCloud cloud;

        bool parseOK = parseCloudXYZ(cloud, lines, success,
                                     csvFilePath, delim);
        if ( !parseOK )
            return;

        qDebug() << "Displaying a cloud of " << success << " points.";
        viewCloud(cloud);
    } else
    {
        XYZICloud cloud;
        bool parseOK = parseCloudXYZI(cloud, lines, success,
                                      csvFilePath, delim, intensityID);

        if ( !parseOK )
            return;

        qDebug() << "Displaying a cloud of " << success << " points.";
        viewCloud(cloud);
    }
}


