//Comments by JLB
#include <QDebug>
#include <QCoreApplication>
#include <QCommandLineParser>

#include "LKHPlanner.h"

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);//Qt->The QCoreApplication class provides an event loop for Qt applications without UI
    QCoreApplication::setApplicationName("UB-ANC Planner");
    QCoreApplication::setApplicationVersion("1.0");

    QCommandLineParser parser;//Qt->The QCommandLineParser class provides a means for handling the command line options
    parser.setApplicationDescription("UB-ANC Planner LKH-D");
    parser.addHelpOption();//Creates a help interface option --help
    parser.addVersionOption();//Creates a version query option --version

    parser.addOptions({//Creation option {{--<opt_name>}, <opt_description>, <opt_default>}
        {{"f", "file"}, "Set mission file containing area information.", "file"},
        {{"r", "resolution"}, "Set resolution of the decomposition in meters.", "resolution", "10"},
        {{"l", "limit"}, "Set optimizer time limit in seconds.", "limit", "1000000000"},
        {{"g", "gap"}, "Set gap to the optimal solution.", "gap", "0.01"},
        {{"a", "lambda"}, "Set distance factor in cost function.", "lambda", "1"},
        {{"m", "gamma"}, "Set turn factor in cost function.", "gamma", "1"},
        {{"k", "kappa"}, "Set maximum capacity for each drone.", "kappa", "1000000000"},
        {{"p", "precision"}, "Set precision for capacity calculation.", "precision", "100"},
        {{"n", "round"}, "Set number of round to run LKH-D.", "round", "1000"},
    });

    parser.process(a);

    if (!parser.isSet("file")) {//we can't process this default file name, considering it's not a real file
        qWarning() << QObject::tr("The area should be set!");
        return 0;
    }
    LKHPlanner* planner = new LKHPlanner();
    planner->setFile(parser.value("file"));//JLB DEBUG manually set file for VScode
    planner->setResolution(parser.value("resolution").toUInt());
    planner->setLimit(parser.value("limit").toUInt());
    planner->setGap(parser.value("gap").toDouble());
    planner->setLambda(parser.value("lambda").toUInt());
    planner->setGamma(parser.value("gamma").toUInt());
    planner->setkappa(parser.value("kappa").toUInt());
    planner->setPrecision(parser.value("precision").toUInt());
    planner->setRound(parser.value("round").toUInt());

    planner->startPlanner();

    a.exec();//Qt->Enters the main event loop and waits until exit() is called. Returns the value that was passed to exit()
}
