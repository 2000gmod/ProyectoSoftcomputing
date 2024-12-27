#include <cstdlib>
#include <ctime>
#include <print>

#include "MainWindow.hpp"


int main(int argc, char* argv[]) {
    srand(time(nullptr));
    
    MainWindow window;
    if (window.Construct(650, 400, 2, 2, false, true, false)) {
        window.Start();
    }
}