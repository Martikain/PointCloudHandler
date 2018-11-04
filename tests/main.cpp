#include <QCoreApplication>
#include <gtest.h>
#include "conversiontests.h"

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
