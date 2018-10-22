#include "pcdwriter.h"
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QFile>
#include <QDir>

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    // Add basic information provided on command line execution
    QCoreApplication::setApplicationName("PCD Handler");
    QCoreApplication::setApplicationVersion("1.0");
    QCommandLineParser parser;
    parser.setApplicationDescription("PCD Handler");
    parser.addHelpOption();
    parser.addVersionOption();

    // Add a positional argument for defining the operation
    parser.addPositionalArgument
            ("command", QCoreApplication::translate
            ("main", "Define the operation with this "
             "positional argument."));

    // Add the options for defining file paths, delimiters etc.
    QCommandLineOption sourceOption
            (QStringList() << "s" << "source",
             QCoreApplication::translate
             ("main", "File path to the source file."),
              "source", "");

    parser.addOption(sourceOption);

    // Create default destination file path
    QString defaultDest(QDir::homePath() + "/data/tmp.pcd");

    QCommandLineOption destOption
            (QStringList() << "d" << "destination",
             QCoreApplication::translate
             ("main", "File path to the destination file."),
              "destination", defaultDest);

    parser.addOption(destOption);

    QCommandLineOption delimOption
            ("delimiter",
             QCoreApplication::translate
             ("main",
              "Delimiter for source file in file conversions. "
              "Default delimiter is space."),
              "delimiter", "");

    parser.addOption(delimOption);

    QCommandLineOption intensityOption(QStringList() << "i" << "intensity",
            QCoreApplication::translate
            ("main",
             "Header identifier for intensity values. If blank, "
             "intensity values will not be stored"),
             "intensity", "");

    parser.addOption(intensityOption);

    QCommandLineOption formatOption
            (QStringList() << "f" << "format",
             QCoreApplication::translate
             ("main",
              "Format for the destination file in conversions. "
              "Either ASCII or BINARY"),
              "format", "ASCII");

    parser.addOption(formatOption);

    // Process the arguments
    parser.process(app);
    const QStringList posArgs(parser.positionalArguments());

    if ( posArgs.size() == 0 )
    {
        qDebug() << "Please provide a command. See --help for "
                    "more information.";
        return 0;
    }

    if ( posArgs.size() != 1 )
    {
        qDebug() << "ERROR: Provide only one command.";
        return 0;
    }

    // Process the 'convert' command
    if ( posArgs.front().toUpper() == "CONVERT" )
    {
        // Get option values
        const QString source(parser.value(sourceOption));
        const QString dest(parser.value(destOption));
        const QString delim(parser.value(delimOption));
        const QString intensityID(parser.value(intensityOption));
        const QString format(parser.value(formatOption));

        if ( source == "" )
        {
            qDebug() << "Please provide the file path to the source file.";
            return 0;
        }

        PCDWriter pcdWriter;
        if ( format.toUpper() == "ASCII" )
            pcdWriter.convertToPCD(source, delim, dest, ASCII, intensityID);
        else if ( format.toUpper() == "BINARY" )
            pcdWriter.convertToPCD(source, delim, dest, binary, intensityID);
        else
        {
            qDebug() << "ERROR: Unknown file conversion format. "
                        "Use either ASCII or BINARY.";
            return 0;
        }
    }

    return 0;
}
