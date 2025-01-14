#include "MainWindow.hpp"
#include "Config.hpp"
#include "TrainingSim.hpp"
#include "Vec2.hpp"

#include <LLThread.hpp>
#include <cmath>
#include <format>

bool MainWindow::OnUserCreate() {
    sAppName = "Drone Training Program";
    Sim.SimDrone = &DefaultDrone;
    SetPixelMode(olc::Pixel::ALPHA);
    return true;
}

bool MainWindow::OnUserUpdate(float dt) {
    Clear(olc::Pixel(0, 0, 32));    
    switch (State) {
        using enum AppState;
        case Menu:
            return MenuUpdate(dt);
        case ManualControl:
            return ManualControlUpdate(dt);
        case AutoControl:
            return AutomaticControlUpdate(dt);
        case Training:
            return TrainingUpdate();
        default:
            return BadStateUpdate();
    }
    

    return true;
}

bool MainWindow::BadStateUpdate() {
    DrawString({10, 10}, "Bad AppState.", olc::RED);
    DrawString({10, 20}, "Press ESC to return to menu.", olc::RED);
    if (GetKey(olc::ESCAPE).bPressed) {
        State = AppState::Menu;
    }
    return true;
}

bool MainWindow::MenuUpdate(float dt) {
    DrawMainMenuBackground(dt);

    DrawCopyright();
    
    DrawString({10, 10}, "Drone training program", olc::YELLOW, 3);
    DrawString({10, 40}, "Main menu");

    auto menuText = R"(
        - [1]   Fly drone manually.
        - [2]   Automatically fly loaded drone.
        - [3]   Go to training mode.


        - [R]   Load random drone into simulation.
        - [L]   Load best drone from last training generation.
        - [B]   Load best drone so far.

        - [ESC] Exit program.
    )";

    DrawString({-50, 90}, menuText);

    if (GetKey(olc::ESCAPE).bPressed) {
        return false;
    }

    else if (GetKey(olc::K1).bPressed) {
        if (Sim.SimDrone != nullptr) Sim.Reset();
        CameraZoom = 0.25;
        CameraFollowDrone = false;
        State = AppState::ManualControl;
    }
    else if (GetKey(olc::K2).bPressed) {
        if (Sim.SimDrone != nullptr) Sim.Reset();
        CameraZoom = 0.25;
        CameraFollowDrone = false;
        State = AppState::AutoControl;
    }
    else if (GetKey(olc::K3).bPressed) {
        //if (Sim.SimDrone != nullptr) Sim.Reset();
        State = AppState::Training;
    }

    else if (GetKey(olc::R).bPressed) {
        DefaultDrone = Drone();
        Sim.SimDrone = &DefaultDrone;
    }
    else if (GetKey(olc::L).bPressed) {
        Sim.SimDrone = &Training.Drones[0];
    }
    else if (GetKey(olc::B).bPressed) {
        Sim.SimDrone = &BestDroneSoFar;
    }
    else if (GetKey(olc::F12).bPressed) {
        State = (AppState) -1;
    }

    return true;
}

bool MainWindow::ManualControlUpdate(float dt) {
    if (GetKey(olc::ESCAPE).bPressed) {
        CameraZoom = 0.25;
        CameraPos = 0;
        State = AppState::Menu;
        return true;
    }

    CameraUpdate();

    DrawGrid();

    DrawString({10, 10}, "Press [ESC] to exit.\nPress [R] to reset.\nUse [LEFT] and [RIGHT] to control drone.");

    if (Sim.SimDrone == nullptr) {
        DrawString({10, 100}, "Physics simulation has no loaded drone.", olc::RED);
        return true;
    }

    if (GetKey(olc::R).bPressed) {
        Sim.Reset();
    }
    
    DrawDrone(*Sim.SimDrone, Sim.RequestedThrust);
    DrawDroneInfoBox();

    Sim.ManualControlStep(GetKey(olc::LEFT).bHeld ? 1.0 : 0.0, GetKey(olc::RIGHT).bHeld ? 1.0 : 0.0, dt);
    
    Sim.DoSimulationStep(dt);
    return true;
}

bool MainWindow::AutomaticControlUpdate(float dt) {
    if (GetKey(olc::ESCAPE).bPressed) {
        CameraZoom = 0.25;
        CameraPos = 0;
        State = AppState::Menu;
        return true;
    }

    CameraUpdate();

    DrawGrid();

    DrawString({10, 10}, "Press [ESC] to exit.\nPress [R] to reset.");

    if (Sim.SimDrone == nullptr) {
        DrawString({10, 100}, "Physics simulation has no loaded drone.", olc::RED);
        return true;
    }

    if (GetKey(olc::R).bPressed) {
        Sim.Reset();
    }

    auto worldMouse = ScreenToWorld(GetMousePos());


    DrawLine(WorldToScreen(Sim.SimDrone->Position), GetMousePos(), olc::PixelF(1.0f, 0.0f, 0.0f, 0.5f), 0xF0F0F0F0);

    FillCircle(GetMousePos(), 5, olc::YELLOW);
    DrawString(GetMousePos() + olc::vi2d {10, -10}, std::format("Target\nx:{: .2f}\ny:{: .2f}", worldMouse.x, worldMouse.y), olc::YELLOW);
    
    DrawDrone(*Sim.SimDrone, Sim.RequestedThrust);
    
    DrawDroneInfoBox();

    Sim.NetworkControlStep(worldMouse, dt);
    
    Sim.DoSimulationStep(dt);
    return true;
}

bool MainWindow::TrainingUpdate() {
    if (GetKey(olc::ESCAPE).bPressed) {
        State = AppState::Menu;
        return true;
    }

    if (GetKey(olc::R).bPressed) {
        Training = TrainingSim();
    }

    if (GetKey(olc::P).bPressed) {
        TrainingPaused = !TrainingPaused;
    }

    if (GetKey(olc::S).bPressed) {
        Training.SaveToFile();
    }

    if (GetKey(olc::L).bPressed) {
        Training.LoadFromFile();
        BestDroneSoFar = Training.Drones[0];
    }

    DrawString({10, 10}, "Hold [ESC] to exit.\nHold [R] to restart training.\nHold [P] to pause/resume training.");

    FP avgPenalty = 0.0;

    auto duration = ll::TimeFunc([this, &avgPenalty] {
        if (!TrainingPaused) avgPenalty = Training.TrainGeneration();
    });

    if (Training.Drones[0].TrainingScore < BestDroneSoFar.TrainingScore) BestDroneSoFar = Training.Drones[0];

    DrawString({10, 60}, std::format("Trained for {} generations.", Training.GenerationsDone));
    DrawString({10, 70}, std::format("Using {} threads for training.", SimulationThreads));
    DrawString({10, 80}, std::format("Took {}", duration));

    DrawString({10, 100}, std::format("Average training loss: {: 5.3f}.", avgPenalty));
    DrawString({10, 110}, std::format("Best drone loss score: {: 5.3f}.", Training.Drones[0].TrainingScore));
    DrawString({10, 150}, std::format("Best drone so far: {: 5.3f}.", BestDroneSoFar.TrainingScore));

    DrawString({10, 200}, "Press [S] to save current generation to checkpoint file.");
    DrawString({10, 210}, "Press [L] to load checkpoint file.");

    if (TrainingPaused) DrawString({10, 250}, "Training paused.", olc::RED, 2);

    Sim.SimDrone = &Training.Drones[0];
    
    return true;
}

void MainWindow::DrawDrone(const Drone& drone, const std::array<FP, 2>& thrust) {
    const auto halfheight = DroneHeight / 2.0;
    const auto halfwidth = DroneWidth / 2.0;

    Vec2 mainBox[] = {
        {halfwidth, halfheight},
        {halfwidth, -halfheight},
        {-halfwidth, -halfheight},
        {-halfwidth, halfheight},
        {0.0, 2 * halfheight},
    };

    for (auto& i : mainBox) {
        i = i.Rotated(drone.DirectionAngle);
        i += drone.Position;
    }

    DrawTriangle(WorldToScreen(mainBox[0]), WorldToScreen(mainBox[1]), WorldToScreen(mainBox[2]));
    DrawTriangle(WorldToScreen(mainBox[3]), WorldToScreen(mainBox[0]), WorldToScreen(mainBox[2]));

    DrawTriangle(WorldToScreen(mainBox[0]), WorldToScreen(mainBox[3]), WorldToScreen(mainBox[4]));

    if (State == AppState::AutoControl) {

        olc::Pixel trainingBoxColor = {255, 255, 0, 128};

        Vec2 trcorners[] = {
            {TrainingMaxCoords, TrainingMaxCoords},
            {TrainingMaxCoords, -TrainingMaxCoords},
            {-TrainingMaxCoords, -TrainingMaxCoords},
            {-TrainingMaxCoords, TrainingMaxCoords},
        };
        
        DrawLine(WorldToScreen(drone.Position + trcorners[0]), WorldToScreen(drone.Position + trcorners[1]), trainingBoxColor);
        DrawLine(WorldToScreen(drone.Position + trcorners[1]), WorldToScreen(drone.Position + trcorners[2]), trainingBoxColor);
        DrawLine(WorldToScreen(drone.Position + trcorners[2]), WorldToScreen(drone.Position + trcorners[3]), trainingBoxColor);
        DrawLine(WorldToScreen(drone.Position + trcorners[3]), WorldToScreen(drone.Position + trcorners[0]), trainingBoxColor);

    }

    if (thrust[0] <= 0.001 && thrust[1] <= 0.01) return;

    auto flameWidthMult = 0.25;

    Vec2 leftThruster[] = {
        {-halfwidth, -halfheight},
        {-halfwidth * flameWidthMult, -halfheight},
        {-halfwidth * (1 + flameWidthMult) / 2.0, -halfheight - 0.25 * thrust[0]}
    };

    Vec2 rightThruster[] = {
        {halfwidth, -halfheight},
        {halfwidth * flameWidthMult, -halfheight},
        {halfwidth * (1 + flameWidthMult) / 2.0, -halfheight - 0.25 * thrust[1]}
    };


    for (auto& i : leftThruster) {
        i = i.Rotated(drone.DirectionAngle);
        i += drone.Position;
    }

    for (auto& i : rightThruster) {
        i = i.Rotated(drone.DirectionAngle);
        i += drone.Position;
    }

    auto color = olc::Pixel(0xFF, 0xA5, 0x00);

    DrawTriangle(WorldToScreen(leftThruster[0]), WorldToScreen(leftThruster[1]), WorldToScreen(leftThruster[2]), color);
    DrawTriangle(WorldToScreen(rightThruster[0]), WorldToScreen(rightThruster[1]), WorldToScreen(rightThruster[2]), color);
}

olc::vi2d MainWindow::WorldToScreen(const Vec2& pos) {
    Vec2 newpos = pos - CameraPos;
    newpos = newpos * CameraZoom;

    auto screenSize = GetScreenSize();
    auto aspectRatio = (float) screenSize.x / screenSize.y;

    newpos.x /= aspectRatio;

    newpos.x = (newpos.x + 1) / 2.0;
    newpos.y = (newpos.y + 1) / 2.0;

    newpos.y = 1.0 - newpos.y;

    olc::vi2d finalPos {(int) (newpos.x * screenSize.x), (int) (newpos.y * screenSize.y)};
    return finalPos;
}

Vec2 MainWindow::ScreenToWorld(const olc::vi2d& pos) {
    auto screenSize = GetScreenSize();
    auto aspectRatio = (float) screenSize.x / screenSize.y;

    Vec2 newpos {(FP) pos.x / screenSize.x, (FP) pos.y / screenSize.y};

    newpos.y = 1.0 - newpos.y;

    newpos.y = 2.0 * newpos.y - 1;
    newpos.x = 2.0 * newpos.x - 1;

    newpos.x *= aspectRatio;

    newpos = newpos / CameraZoom;
    return newpos + CameraPos;
}

void MainWindow::DrawCopyright() {
    auto msg = "Made by Diego Vasquez W.\nIPD434 - SEMINARIO DE SOFTCOMPUTING\nUTFSM";
    DrawString({10, ScreenHeight() - 40}, msg, olc::DARK_YELLOW);
}

void MainWindow::DrawGrid() {
    auto minorLineColor = olc::DARK_GREY;
    minorLineColor.a = 128;

    auto axisColor = olc::DARK_GREY;

    auto upperLeft = ScreenToWorld({0, 0});
    auto lowerRight = ScreenToWorld(GetScreenSize());

    auto upperXBound = std::ceil(lowerRight.x);
    auto lowerXBound = std::floor(upperLeft.x);

    auto upperYBound = std::ceil(upperLeft.y);
    auto lowerYBound = std::floor(lowerRight.y);

    for (FP y = lowerYBound; y <= upperYBound; y += 1) {
        DrawLine(WorldToScreen({lowerXBound, y}), WorldToScreen({upperXBound, y}), minorLineColor);
    }

    for (FP x = lowerXBound; x <= upperXBound; x += 1) {
        DrawLine(WorldToScreen({x, upperYBound}), WorldToScreen({x, lowerYBound}), minorLineColor);
    }

    DrawLine(WorldToScreen({lowerXBound, 0}), WorldToScreen({upperXBound, 0}), axisColor);
    DrawLine(WorldToScreen({0, upperYBound}), WorldToScreen({0, lowerYBound}), axisColor);
}

void MainWindow::DrawDroneInfoBox() {
    auto bgColor = olc::BLACK;
    bgColor.a = 200;

    auto width = 250;
    auto height = 70;

    olc::vi2d topLeft ={0, ScreenHeight() - height};

    FillRect(topLeft, {width, height}, bgColor);
    
    constexpr auto textFormat = R"(
    DRONE INFO

    Position: ({: .2f}, {: .2f})
    Velocity: ({: .2f}, {: .2f})
    Angle: {: 06.1f}
    Angular Velocity: {: 06.1f}
    )";

    const Drone& drone = *Sim.SimDrone;

    auto text = std::format(
        textFormat, 
        drone.Position.x, drone.Position.y,
        drone.Velocity.x, drone.Velocity.y,
        drone.DirectionAngle * 180 / std::numbers::pi,
        drone.AngularVelocity * 180 / std::numbers::pi
    );

    DrawString(topLeft + olc::vi2d {-20, 10}, text);
}

void MainWindow::CameraUpdate() {
    if (GetKey(olc::L).bPressed) {
        CameraZoom *= 2;
    }

    if (GetKey(olc::K).bPressed) {
        CameraZoom /= 2;
    }

    if (GetKey(olc::F1).bPressed) {
        CameraPos = {0, 0};
    }

    if (GetKey(olc::V).bPressed) {
        CameraFollowDrone = !CameraFollowDrone;
    }


    if (CameraFollowDrone) {
        CameraPos = Sim.SimDrone->Position;
    }
}

void MainWindow::DrawMainMenuBackground(float dt) {
    MenuBackgroundAngle += 0.65 * dt;
    const auto halfheight = DroneHeight * 2.0;
    const auto halfwidth = DroneWidth * 2.0;

    Vec2 positionOffset = {3.8, 0.0};

    Vec2 mainBox[] = {
        {halfwidth, halfheight},
        {halfwidth, -halfheight},
        {-halfwidth, -halfheight},
        {-halfwidth, halfheight},
        {0.0, 2 * halfheight},
    };

    for (auto& i : mainBox) {
        i = i.Rotated(MenuBackgroundAngle);
        i += positionOffset;
    }

    DrawTriangle(WorldToScreen(mainBox[0]), WorldToScreen(mainBox[1]), WorldToScreen(mainBox[2]));
    DrawTriangle(WorldToScreen(mainBox[3]), WorldToScreen(mainBox[0]), WorldToScreen(mainBox[2]));

    DrawTriangle(WorldToScreen(mainBox[0]), WorldToScreen(mainBox[3]), WorldToScreen(mainBox[4]));

    auto flameWidthMult = 0.25;
    auto freq1 = 16.72;
    auto freq2 = 18.23;

    Vec2 leftThruster[] = {
        {-halfwidth, -halfheight},
        {-halfwidth * flameWidthMult, -halfheight},
        {-halfwidth * (1 + flameWidthMult) / 2.0, -halfheight - 0.25 * std::abs(0.25 + std::sin(MenuBackgroundAngle * freq1))}
    };

    Vec2 rightThruster[] = {
        {halfwidth, -halfheight},
        {halfwidth * flameWidthMult, -halfheight},
        {halfwidth * (1 + flameWidthMult) / 2.0, -halfheight - 0.25 * std::abs(0.25 + std::cos(MenuBackgroundAngle * freq2))}
    };


    for (auto& i : leftThruster) {
        i = i.Rotated(MenuBackgroundAngle);
        i += positionOffset;
    }

    for (auto& i : rightThruster) {
        i = i.Rotated(MenuBackgroundAngle);
        i += positionOffset;
    }

    auto color = olc::Pixel(0xFF, 0xA5, 0x00);

    DrawTriangle(WorldToScreen(leftThruster[0]), WorldToScreen(leftThruster[1]), WorldToScreen(leftThruster[2]), color);
    DrawTriangle(WorldToScreen(rightThruster[0]), WorldToScreen(rightThruster[1]), WorldToScreen(rightThruster[2]), color);
}