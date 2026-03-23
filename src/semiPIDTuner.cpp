#include "semiPIDTuner.h"
#include <cstdio>

//distances
const int PIDTuner::DRIVE_DISTANCES[] = {3, 6, 12, 24, 36, 48, 72};
const int PIDTuner::NUM_DRIVE_DISTANCES = 7;
const int PIDTuner::TURN_DISTANCES[] = {5, 10, 30, 45, 90, 180, 270, 360};
const int PIDTuner::NUM_TURN_DISTANCES = 8;

//constructor (set defaults)
PIDTuner::PIDTuner(Drive& chassis) : chassis(chassis){
    driveParameters.kP = 0.0f;
    driveParameters.kI = 0.0f;
    driveParameters.kD = 0.0f;
    driveParameters.settleError = 1.0f;
    driveParameters.timeToSettle = 200;
    driveParameters.endTime = 2500;

    turnParameters.kP = 0.0f;
    turnParameters.kI = 0.0f;
    turnParameters.kD = 0.0f;
    turnParameters.settleError = 1.0f;
    turnParameters.timeToSettle = 200;
    turnParameters.endTime = 2500;
}

//hold to repeat increase or decrease values
bool PIDTuner::shouldFire(int h) const {
    if (h == 1) return true;
    if (h < 8) return false;
    if (h < 19) return (h % 5 == 0);
    if (h < 38) return (h% 2 == 0);
    return true;
}

//button helpers
PIDTuner::BtnSnapshot PIDTuner::rawButtons() {
    BtnSnapshot b;
    b.A = Controller1.ButtonA.pressing();
    b.B     = Controller1.ButtonB.pressing();
    b.R1     = Controller1.ButtonR1.pressing();
    b.L1     = Controller1.ButtonL1.pressing();
    b.R2    = Controller1.ButtonR2.pressing();
    b.Up    = Controller1.ButtonUp.pressing();
    b.Down  = Controller1.ButtonDown.pressing();
    b.Left  = Controller1.ButtonLeft.pressing();
    b.Right = Controller1.ButtonRight.pressing();
    return b;
}

PIDTuner::BtnSnapshot PIDTuner::edgeButtons() {
    BtnSnapshot cur = rawButtons();
    BtnSnapshot e;
    e.A     = cur.A     && !prevBtns.A;
    e.B     = cur.B     && !prevBtns.B;
    e.R1     = cur.R1     && !prevBtns.R1;
    e.L1     = cur.L1     && !prevBtns.L1;
    e.R2    = cur.R2      && !prevBtns.R2;
    e.Up    = cur.Up    && !prevBtns.Up;
    e.Down  = cur.Down  && !prevBtns.Down;
    e.Left  = cur.Left  && !prevBtns.Left;
    e.Right = cur.Right && !prevBtns.Right;
    prevBtns = cur;
    return e;
}

//helpers
PIDTuner::ParameterSet& PIDTuner::activeParams() {
    return (currentMode == MODE_DRIVE) ? driveParameters : turnParameters;
}

const char* PIDTuner::modeLabel() const {
    return (currentMode == MODE_DRIVE) ? "DrivePID" : "TurnPID";
}

const int* PIDTuner::activeDistances() const{
    return (currentMode == MODE_DRIVE) ? DRIVE_DISTANCES : TURN_DISTANCES;
}

int PIDTuner::activeNumDistances() const{
    return (currentMode == MODE_DRIVE) ? NUM_DRIVE_DISTANCES : NUM_TURN_DISTANCES;
}

void PIDTuner::applyParamsToChassis(){
    chassis.setDriveConstants(driveParameters.kP, driveParameters.kI, driveParameters.kD, driveParameters.settleError, driveParameters.timeToSettle, driveParameters.endTime);
    chassis.setTurnConstants(turnParameters.kP, turnParameters.kI, turnParameters.kD, turnParameters.settleError, turnParameters.timeToSettle, turnParameters.endTime);
}

//Open specific edit screen for a field
void PIDTuner::openEditScreen() {
    ParameterSet& p = activeParams();
 
    switch (fieldSelector) {
        case 0:
            editLabel      = "P";
            editFloatValue   = p.kP;
            editFineStep   = 0.01f;
            editCoarseStep = 0.1f;
            editFloatMin   = 0.0f;
            editFloatMax   = 99.99f;
            currentScreen  = SCR_EDIT_FLOAT;
            break;
        case 1:
            editLabel      = "I";
            editFloatValue   = p.kI;
            editFineStep   = 0.01f;
            editCoarseStep = 0.1f;
            editFloatMin   = 0.0f;
            editFloatMax   = 99.99f;
            currentScreen  = SCR_EDIT_FLOAT;
            break;
        case 2:
            editLabel      = "D";
            editFloatValue   = p.kD;
            editFineStep   = 0.01f;
            editCoarseStep = 0.1f;
            editFloatMin   = 0.0f;
            editFloatMax   = 99.99f;
            currentScreen  = SCR_EDIT_FLOAT;
            break;
        case 3:
            editLabel      = "Err";
            editFloatValue   = p.settleError;
            editFineStep   = 0.05f;
            editCoarseStep = 0.5f;
            editFloatMin   = 0.0f;
            editFloatMax   = 99.99f;
            currentScreen  = SCR_EDIT_FLOAT;
            break;
        case 4:
            editLabel       = "Settle";
            editIntVal      = p.timeToSettle;
            editFineStepI   = 25;
            editCoarseStepI = 100;
            editIntMin      = 0;
            editIntMax      = 9999;
            currentScreen   = SCR_EDIT_INT;
            break;
        case 5:
            editLabel       = "End";
            editIntVal      = p.endTime;
            editFineStepI   = 25;
            editCoarseStepI = 100;
            editIntMin      = 0;
            editIntMax      = 9999;
            currentScreen   = SCR_EDIT_INT;
            break;
    }
 
    // Reset hold timers when entering any edit screen
    holdUp = holdDown = 0;
}

void PIDTuner::saveEditScreen() {
    ParameterSet& p = activeParams();
    switch (fieldSelector) {
        case 0: p.kP           = editFloatValue; break;
        case 1: p.kI           = editFloatValue; break;
        case 2: p.kD           = editFloatValue; break;
        case 3: p.settleError = editFloatValue; break;
        case 4: p.timeToSettle = editIntVal;   break;
        case 5: p.endTime      = editIntVal;   break;
    }
}

//Controller Screen
void PIDTuner::drawMain() {
    ParameterSet& p = activeParams();
    static const char* NAMES[6] = {"P", "I", "D", "Err", "Settle", "End"};
 
    Controller1.Screen.clearScreen();
 
    // Row 1: mode + distance
    Controller1.Screen.setCursor(1, 1);
    const int* distances = activeDistances();
    const char* unit = (currentMode == MODE_DRIVE) ? "in" : "deg";
    Controller1.Screen.print("%s [%d %s]", modeLabel(), distances[distIndx], unit);
 
    // Row 2: currently selected field with its value
    Controller1.Screen.setCursor(2, 1);
    if (fieldSelector <= 3) {
        float vals[4] = { p.kP, p.kI, p.kD, p.settleError };
        Controller1.Screen.print("> %s  %.4f", NAMES[fieldSelector], (double)vals[fieldSelector]);
    } else {
        int vals[2] = { p.timeToSettle, p.endTime };
        Controller1.Screen.print("> %s  %d ms", NAMES[fieldSelector], vals[fieldSelector - 4]);
    }
 
    // Row 3: hints
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("A=run UD=sel LR=dist B=edit");
}
 
void PIDTuner::drawEditFloat() {
    Controller1.Screen.clearScreen();
 
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Edit %s", editLabel);
 
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%.4f", (double)editFloatValue);
 
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("UD=%.2f AX=%.1f B=save",
                             (double)editFineStep,
                             (double)editCoarseStep);
}
 
void PIDTuner::drawEditInt() {
    Controller1.Screen.clearScreen();
 
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Edit %s", editLabel);
 
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%d ms", editIntVal);
 
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("UD=%dms AX=%dms B=save",
                             editFineStepI, editCoarseStepI);
}

//run test
void PIDTuner::runTest() {
    applyParamsToChassis();
    
    const int* distances = activeDistances();
    float dist   = (float)distances[distIndx];
    float target = testGoForward ? dist : -dist;
    testGoForward = !testGoForward;
 
    const char* unit = (currentMode == MODE_DRIVE) ? "in" : "deg";
 
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("%s test", modeLabel());
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Target: %.1f %s", (double)target, unit);
 
    if (currentMode == MODE_DRIVE) {
        chassis.driveDistanceWithOdom(target);
    } else {
        chassis.turn(target);
    }
 
    chassis.brake();
 
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Done!");
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Next: %s %.0f %s",
                             testGoForward ? "FWD" : "BWD",
                             (double)dist, unit);
    wait(1200, msec);
}

//main loop
void PIDTuner::run() {
    bool redraw = true;
    int maxDist;
 
    while (true) {
        BtnSnapshot edge = edgeButtons();
        BtnSnapshot raw  = rawButtons();
        bool changed     = false;
        
 
        switch (currentScreen) {
 
            // ── MAIN ──────────────────────────────────────────────────────
            case SCR_MAIN:
                if (edge.R2) { runTest(); redraw = true; break; }
 
                // Move field selection — clamped
                if (edge.Left   && fieldSelector > 0) { fieldSelector--; changed = true; }
                if (edge.Right && fieldSelector < 5) { fieldSelector++; changed = true; }
 
                // Cycle distance — clamped
                maxDist = activeNumDistances();
                if (edge.Up && distIndx < maxDist - 1) { distIndx++; changed = true; }
                if (edge.Down  && distIndx > 0)                  { distIndx--; changed = true; }
 
                // Switch mode
                if (edge.R1) { 
                    currentMode = MODE_DRIVE; 
                    if (distIndx >= NUM_DRIVE_DISTANCES)
                        distIndx = NUM_DRIVE_DISTANCES -1;
                    changed = true; 
                }
                if (edge.L1) { 
                    currentMode = MODE_TURN;  
                    if (distIndx >= NUM_TURN_DISTANCES)
                        distIndx = NUM_TURN_DISTANCES -1;
                    changed = true; 
                }
 
                // Open edit screen
                if (edge.A) { openEditScreen(); redraw = true; break; }
 
                if (changed) redraw = true;
                if (redraw)  { drawMain(); redraw = false; }
                break;
 
            // ── EDIT FLOAT ────────────────────────────────────────────────
            case SCR_EDIT_FLOAT:
                // Increment hold timers for held buttons, reset if released
                holdUp   = raw.Up   ? holdUp   + 1 : 0;
                holdDown = raw.Down ? holdDown + 1 : 0;
 
                if (shouldFire(holdUp)) {
                    if (holdUp < 30){
                        editFloatValue += editFineStep;
                    }else{
                        editFloatValue += editCoarseStep;
                    }
                    if (editFloatValue > editFloatMax) editFloatValue = editFloatMax;
                    changed = true;
                }
                if (shouldFire(holdDown)) {
                    if (holdDown < 30){
                        editFloatValue -= editFineStep;
                    }else{
                        editFloatValue -= editCoarseStep;
                    }
                    if (editFloatValue < editFloatMin) editFloatValue = editFloatMin;
                    changed = true;
                }
 
                if (edge.B) {
                    saveEditScreen();
                    currentScreen = SCR_MAIN;
                    redraw = true;
                    break;
                }
 
                if (changed) redraw = true;
                if (redraw)  { drawEditFloat(); redraw = false; }
                break;
 
            // ── EDIT INT ──────────────────────────────────────────────────
            case SCR_EDIT_INT:
                holdUp   = raw.Up   ? holdUp   + 1 : 0;
                holdDown = raw.Down ? holdDown + 1 : 0;
 
                if (shouldFire(holdUp)) {
                    if (holdUp < 30){
                        editIntVal += editFineStepI;
                    }else{
                        editIntVal += editCoarseStepI;
                    }
                    if (editIntVal > editIntMax) editIntVal = editIntMax;
                    changed = true;
                }
                if (shouldFire(holdDown)) {
                    if (holdDown < 30){
                        editIntVal -= editFineStepI;
                    }else{
                        editIntVal -= editCoarseStepI;
                    }
                    if (editIntVal < editIntMin) editIntVal = editIntMin;
                    changed = true;
                }

 
                if (edge.B) {
                    saveEditScreen();
                    currentScreen = SCR_MAIN;
                    redraw = true;
                    break;
                }
 
                if (changed) redraw = true;
                if (redraw)  { drawEditInt(); redraw = false; }
                break;
        }
 
        wait(40, msec); // ~25 polls per second
    }
}
