#include "core/Application.h"
#include <QLoggingCategory>
#include <iostream>

Q_LOGGING_CATEGORY(mainApp, "branchforge.main")

int main(int argc, char *argv[]) {
    try {
        BranchForge::Core::Application app(argc, argv);
        return app.run();
    } catch (const std::exception& e) {
        qCCritical(mainApp) << "Unhandled exception:" << e.what();
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        qCCritical(mainApp) << "Unknown exception occurred";
        std::cerr << "Fatal error: Unknown exception" << std::endl;
        return -1;
    }
}