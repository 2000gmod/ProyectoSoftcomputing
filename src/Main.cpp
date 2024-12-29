#include <cstdlib>
#include <ctime>
#include <print>

#include "MainWindow.hpp"


int main() {
    srand(time(nullptr));

    int pixSize = 2;
    int width = 1600;
    int height = 900;
    
    MainWindow window;
    if (window.Construct(width / pixSize, height / pixSize, pixSize, pixSize, false, true, false)) {
        window.Start();
    }
}