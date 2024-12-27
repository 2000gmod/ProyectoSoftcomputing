#pragma once

#include "Drone.hpp"
#include "PhysicsSim.hpp"
#include "TrainingSim.hpp"
#include "Vec2.hpp"
#include "olcPGE.hpp"

class MainWindow : public olc::PixelGameEngine {
    private:
        enum class AppState {
            Menu,
            ManualControl,
            AutoControl,
            Training,
        };

        AppState State = AppState::Menu;
        Drone DefaultDrone;
        Drone BestDroneSoFar;

    public:
        PhysicsSim Sim;
        TrainingSim Training;

        const Vec2 CameraPos = {0.0, 0.0};
        const float CameraZoom = 0.25;

        bool OnUserCreate() override;
        bool OnUserUpdate(float) override;

        bool BadStateUpdate();
        bool MenuUpdate();
        bool ManualControlUpdate(float);
        bool AutomaticControlUpdate(float);
        bool TrainingUpdate();

        void DrawDrone(const Drone& drone, const std::array<FP, 2>& thrust);
        olc::vi2d WorldToScreen(const Vec2& pos);
        Vec2 ScreenToWorld(const olc::vi2d& pos);

        void DrawCopyright();
        void DrawGrid();
        void DrawDroneInfoBox();
        
};