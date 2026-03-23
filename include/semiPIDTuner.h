#pragma once
#include "vex.h"
#include "drive.h"

class PIDTuner {
    public:
        explicit PIDTuner(Drive& chassis);

        void run();

    private:
        Drive& chassis;

        struct ParameterSet {
            float kP = 0.0f;
            float kI = 0.0f;
            float kD = 0.0f;
            float settleError = 1.0f;
            int timeToSettle = 250;
            int endTime = 3000;
        };

        ParameterSet driveParameters;
        ParameterSet turnParameters;

        //UI
        enum Screen {SCR_MAIN, SCR_EDIT_FLOAT, SCR_EDIT_INT};
        enum Mode {MODE_DRIVE, MODE_TURN};

        Screen currentScreen = SCR_MAIN;
        Mode currentMode = MODE_DRIVE;

        //Fields on main Screen
        int fieldSelector = 0;

        bool testGoForward = true;

        static const int DRIVE_DISTANCES[];
        static const int NUM_DRIVE_DISTANCES;
        static const int TURN_DISTANCES[];
        static const int NUM_TURN_DISTANCES;
        int distIndx = 2;

        //SCR_EDIT_FLOAT values
        float editFloatValue = 0.0f;
        float editFineStep = 0.01f;
        float editCoarseStep = 0.1f;
        float editFloatMin = 0.0f;
        float editFloatMax = 99.99f;

        //SCR_EDIT_INT values
        int editIntVal = 0;
        int editFineStepI = 25;
        int editCoarseStepI = 100;
        int editIntMin = 0;
        int editIntMax = 9999;

        //labels on edit screen
        const char* editLabel = "";

        //hold timers
        int holdUp = 0;
        int holdDown = 0;

        bool shouldFire(int holdCount) const;

        struct BtnSnapshot {bool A, B, R1, L1, R2, Up, Down, Left, Right;};
        BtnSnapshot prevBtns = {};

        BtnSnapshot edgeButtons();

        BtnSnapshot rawButtons();

        ParameterSet& activeParams();
        const char* modeLabel() const;

        const int* activeDistances() const;
        int activeNumDistances() const;

        void applyParamsToChassis();

        void openEditScreen();
        void saveEditScreen();

        void drawMain();
        void drawEditFloat();
        void drawEditInt();

        void runTest();

};