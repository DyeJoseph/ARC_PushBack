#include "autoPIDTuner.h"
#include <algorithm>
#include <cmath>
//Constructor

AutoPIDTuner::AutoPIDTuner(Drive& drive, tunerConfig config): _drive(drive), _cfg(config){}


//Main Controller
PIDGains AutoPIDTuner::tuneDrive(){
  std::cout << "AutoPIDTuner: Drive Step Test" << std::endl;
  std::cout << "Running 3 Forward and 3 Reverse Runs" << std::endl;

  std::vector<stepResponseData> allRuns;
  

  //Do Runs
  for (int i=0; i < 3; i++){
    std::cout << "Forward " << i << "/3 Running" << std::endl;
    wait(1500, msec);
    stepResponseData run = runStepTest(_cfg.stepVoltage);
    analyzeResponse(run);
    allRuns.push_back(run);
  }

  for (int i=0; i < 3; i++){
    std::cout << "Reverse " << i << "/3 Running" << std::endl;
    wait(1500, msec);
    stepResponseData run = runStepTest(-_cfg.stepVoltage);
    analyzeResponse(run);
    allRuns.push_back(run);
  }

  //Average Runs
  std::cout << "Averaging all 6 runs" << std:: endl;
  stepResponseData averaged = averageStepData(allRuns);

  //Calculate gains
  std::cout << "Calculating PID Gains" << std::endl;
  PIDGains gains;
  if (_cfg.method == "ZN"){
    gains = calculateGainsZN(averaged);
  }else{
    gains = calculateGainsITAE(averaged);
  }

  printGains(gains, averaged);
  _lastData = averaged;
  _lastGains = gains;
  return gains;
}

//Driving test
stepResponseData AutoPIDTuner::runStepTest(double voltage){
  stepResponseData data;
  _drive.setPosition(0,0,0);
  double pos = 0.0;
  wait(100, msec);

  double startX = _drive.chassisOdometry.getXPosition();
  double startY = _drive.chassisOdometry.getYPosition();
  double startHeading = _drive.chassisOdometry.getHeading();

  timer runTimer;
  runTimer.reset();

  _drive.driveMotors(voltage, voltage);

  while (true) {
    double elapsedTime = runTimer.time(msec);

    _drive.updatePosition();
    double dx = _drive.chassisOdometry.getXPosition() - startX;
    double dy = _drive.chassisOdometry.getYPosition() - startY;
    pos = sqrt(dx * dx + dy * dy);

    double heading = _drive.chassisOdometry.getHeading() - startHeading;

    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    data.time.push_back(elapsedTime);

    data.position.push_back(pos);

    //safety
    if (elapsedTime > _cfg.stepDuration) break;
    if (elapsedTime >= _cfg.maxRunTimeMs){
      std::cout << "Max run time" << std::endl;
      break;
    }
    if (pos > _cfg.maxDistanceIn){
      std::cout << "Max distance reached" << std::endl;
      break;
    }
    if (fabs(heading) > 15){
      std::cout << "Heading drift > 15 degrees" << std::endl;
      break;
    }

    wait(_cfg.sampleIntervalMs, msec);
  }

  _drive.brake(brake);
  wait(300, msec);

  std::cout << " Number of Samples" << (int)data.time.size() << std::endl;
  return data;
}

stepResponseData AutoPIDTuner::averageStepData(const std::vector<stepResponseData>& runs){
  std::vector<double> riseTimes;
  for (const auto& r : runs){
    riseTimes.push_back(r.riseTime);
  }
  std::sort(riseTimes.begin(), riseTimes.end());
  double medianRise = riseTimes[riseTimes.size() / 2];

  std::vector<stepResponseData> goodRuns;
  for (const auto& r:runs){
    if (fabs(r.riseTime - medianRise) / medianRise < 0.40){
      goodRuns.push_back(r);
    }

  }
  
  stepResponseData avg;
  int n = (int)goodRuns.size();
  
  for (const auto& r : goodRuns){
    avg.peakVelocity += r.peakVelocity;
    avg.riseTime += r.riseTime;
    avg.settlingTime += r.settlingTime;
    avg.overShootPct += r.overShootPct;
    avg.dcGain += r.dcGain;
    avg.timeConstant += r.timeConstant;
  }

  avg.peakVelocity /= n;
  avg.riseTime /= n;
  avg.settlingTime /= n;
  avg.overShootPct /= n;
  avg.dcGain /= n;
  avg.timeConstant /= n;


  //Get the best velocity closest to average. Use same time/pos/velocity
  double bestDiff = 1e9;
  int bestIdx = 0;
  for (int i = 0; i < n; i++){
    double diff = fabs(runs[i].peakVelocity - avg.peakVelocity);
    if (diff < bestDiff){
      bestDiff = diff;
      bestIdx = i;
    }
  }

  avg.time = runs[bestIdx].time;
  avg.position = runs[bestIdx].position;
  avg.velocity = runs[bestIdx].velocity;

  //print every run summary
  std::cout << std::fixed << std::setprecision(2) << std::endl << "  Run breakdown:" << std::endl;
    for (int i = 0; i < n; i++) {
        std::cout << "  [" << (i < 3 ? "F" : "R") << (i % 3 + 1) << "]"
                  << "  Peak: " << std::setprecision(2) << runs[i].peakVelocity << " in/s"
                  << "  Rise: " << std::setprecision(1) << runs[i].riseTime     << " ms"
                  << "  Tau: "  << runs[i].timeConstant << " ms" << std::endl;
    }
    std::cout << "  -----------" << std::endl
              << "  Avg Peak: " << std::setprecision(2) << avg.peakVelocity << " in/s"
              << "  Avg Rise: " << std::setprecision(1) << avg.riseTime     << " ms"
              << "  Avg Tau: "  << avg.timeConstant << " ms" << std::endl
              << "-------------------------------------" << std::endl;

    return avg;
}

void AutoPIDTuner::analyzeResponse(stepResponseData& data){
  int n = (int)data.position.size();
  if (n < 3) {
    std::cout << "Not enough samples" << std::endl;
    return;
  }


  //velocity via central difference
  data.velocity.resize(n, 0.0);
  for (int i = 1; i < n - 1; i++){
    double dt = (data.time[i+1] - data.time[i-1]) / 1000.0; //ms to sec
    double dp = data.position[i+1] - data.position[i-1];
    if (dt > 0){
      data.velocity[i] = dp / dt;
    }
  }

  data.velocity[0] = data.velocity[1];
  data.velocity[n-1] = data.velocity[n-2];


  //3 point moving average
  std::vector<double> smoothV = data.velocity;
  for (int i = 1; i < n-1; i++){
    smoothV[i] = (data.velocity[i-1] + data.velocity[i] + data.velocity[i+1]) /3;
  }
  data.velocity = smoothV;

  //peak velocity
  data.peakVelocity = *std::max_element(data.velocity.begin(), data.velocity.end());
  //DC gain: velocity per volt
  data.dcGain = data.peakVelocity / _cfg.stepVoltage;

  double v10 = .10 * data.peakVelocity;
  double v90 = .90 * data.peakVelocity;
  double t10 = -1, t90 = -1;
  for (int i = 0; i < n; i++){
    if (t10 < 0 && data.velocity[i] >= v10){
      t10 = data.time[i];
    }
    if (t90 < 0 && data.velocity[i] >= v90){
      t90 = data.time[i];
    }
  }
  data.riseTime = (t10 >= 0 && t90 >= 0) ? (t90 - t10) : 200.0;


  //time constant tau: time to reach 63.2% of peak
  double v63 = .632 * data.peakVelocity;
  data.timeConstant = data.riseTime;
  for (int i = 0; i < n; i++){
    if (data.velocity[i] >= v63){
      data.timeConstant = data.time[i];
      break;
    }
  }
  
  //settling time, last sample outside 5% of peak
  double band = 0.05 * data.peakVelocity;
  data.settlingTime = data.time.back();
  for (int i = n - 1; i >= 0; i--){
    if (fabs(data.velocity[i] - data.peakVelocity) > band){
      data.settlingTime = data.time[i];
      break;
    }
  }

  //overshoot
  double finalVel = data.velocity[n-1];
  data.overShootPct = (finalVel > 0) ? ((data.peakVelocity - finalVel) / finalVel) * 100.0 : 0.0;
}

//ITAE tuning
PIDGains AutoPIDTuner::calculateGainsITAE(const stepResponseData& data){
  PIDGains gains;

  double K = data.dcGain;
  double tau = data.timeConstant / 1000.0;
  double L = (data.riseTime / 1000.0) * .15;

  if ( K <= 0 || tau < 0 || L <= 0){
    std::cout << "Bad info, using safe values instead";
    gains.kP = 1.0; gains.kI = 0.01; gains.kD = 0.05;
    return gains;
  }

  double ratio = L/tau;
  gains.kP = (.965 / (K * L)) * std::pow(ratio, -.855);
  double Ti = 2.39 * tau * std::pow(ratio, 0.381);
  double Td = 0.381 * tau * std::pow(ratio, 0.995);
  gains.kI = gains.kP / Ti;
  gains.kD = gains.kP * Td;

  // gains.kP = 1.0 / (K * L);
  // gains.kI = gains.kP * 0.001;
  // gains.kD = gains.kP * tau * 0.5;

  //Add safety values for kI, kD, and kP if needed

  return gains;
}

//ZN Tuning
PIDGains AutoPIDTuner::calculateGainsZN(const stepResponseData& data) {
    PIDGains gains;

    double K   = data.dcGain;
    double tau = data.timeConstant / 1000.0;
    double L   = (data.riseTime / 1000.0) * 0.15;

    if (K <= 0 || tau <= 0 || L <= 0) {
        gains.kP = 1.0; gains.kI = 0.01; gains.kD = 0.05;
        return gains;
    }

    // gains.kP = (1.2 * tau) / (K * L);
    // gains.kI = gains.kP / (2.0 * L);
    // gains.kD = gains.kP * (0.5 * L);

    gains.kP = 1.2 / (K * L);
    gains.kI = gains.kP * 0.002;
    gains.kD = gains.kP * tau * 0.6;

    //Add safety values for kI, kD, and kP if needed

    return gains;
}

void AutoPIDTuner::printGains(const PIDGains& gains, const stepResponseData& data) {
    std::cout << std::fixed
              << "-------------------------------------"                                    << std::endl
              << "  Method:    " << _cfg.method                                            << std::endl
              << "  Peak Vel:  " << std::setprecision(2) << data.peakVelocity << " in/s"  << std::endl
              << "  Rise Time: " << std::setprecision(1) << data.riseTime     << " ms"    << std::endl
              << "  Tau:       " << std::setprecision(1) << data.timeConstant << " ms"    << std::endl
              << "  DC Gain:   " << std::setprecision(4) << data.dcGain                   << std::endl
              << "-------------------------------------"                                    << std::endl
              << "  kP = "       << std::setprecision(4) << gains.kP                      << std::endl
              << "  kI = "       << std::setprecision(6) << gains.kI                      << std::endl
              << "  kD = "       << std::setprecision(4) << gains.kD                      << std::endl
              << "-------------------------------------"                                    << std::endl
              << "  Paste into your chassis config:"                                       << std::endl
              << "  drive.setDrivePID("
                  << std::setprecision(4) << gains.kP << ", "
                  << std::setprecision(6) << gains.kI << ", "
                  << std::setprecision(4) << gains.kD << ");"                             << std::endl
              << "=====================================" << std::endl;
}

