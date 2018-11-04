#include "conversiontests.h"

namespace pcl
{
bool operator==(const pcl::PointXYZI &lhs, const pcl::PointXYZI &rhs)
{
    double xDiff = fabs(lhs.x - rhs.x);
    double yDiff = fabs(lhs.y - rhs.y);
    double zDiff = fabs(lhs.z - rhs.z);
    double iDiff = fabs(lhs.intensity - rhs.intensity);

    return xDiff < 0.001 &&
           yDiff < 0.001 &&
           zDiff < 0.001 &&
           iDiff < 0.001;
}
} // namespace pcl

TEST (ConversionTest, SimpleSuccecss)
{
    QString testDir = QDir::homePath();
    testDir += "/PointCloudHandler/tests";

    QString inputFilePath = testDir;
    inputFilePath += "/inputs/simpleSuccess.csv";
    pcl::PointCloud<pcl::PointXYZI> inputCloud;

    pcl::PointCloud<pcl::PointXYZI> outputCloud;
    std::string cloudFilePath = testDir.toStdString();
    cloudFilePath += "/outputs/simpleSuccess_ascii.pcd";
    pcl::io::loadPCDFile(cloudFilePath , outputCloud);

    bool success = CSVtoPCDwithIntensity(inputCloud, inputFilePath,
                                         TEST_DELIMITER, TEST_INTENSITY_ID);

    EXPECT_EQ(true, success);

    size_t outputCloudSize = outputCloud.size();
    size_t inputCloudSize = inputCloud.size();

    ASSERT_EQ(outputCloudSize, inputCloudSize);

    for ( size_t i = 0; i < inputCloudSize; i++ )
    {
        if ( i >= outputCloudSize )
        {
            ASSERT_EQ(true, false);
        }

        ASSERT_EQ(inputCloud.at(i), outputCloud.at(i));
    }
}

