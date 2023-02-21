#include <iostream>

#include "app.h"

int main() {
    crl::app::locomotion::App app;
    app.setCallbacks();
    app.run();

    return 0;
}
