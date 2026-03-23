#pragma once
#include "vex.h"
#include "Drive.h"

//Stores P, I, and D values. Also outputes what the values are.
struct PIDGains {   
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  std::string toString() const{
    std::ostringstream oss;
    oss << "kP = " << kP << " | kI = " << kI << " | kD = " << kD;
    return oss.str();
  }
};

struct stepResponseData {
  std::vector<double> time;       //  ms timestamps
  std::vector<double> position;   //  inches
  std::vector<double> velocity;   //  inches/sec  
  double peakVelocity = 0.0;      //  maxVelocity  
  double riseTime = 0.0;          //  ms: 10% to 90% of peak  
  double settlingTime = 0.0;      //  ms: within 5% of peak 
  double overShootPct = 0.0;      //  %  
  double dcGain = 0.0;            //  peak velocity / step voltage
  double timeConstant = 0.0;      //  tau
};

struct tunerConfig {
  //Step Parameters
  double stepVoltage = 9.0;        // Volts
  double stepDuration = 1500;      // ms: How long to run test
  double sampleIntervalMs = 10;    // ms: How often to collect samples

  //Drive Distance for validation
  double validationDistance = 24.0;

  //Tuning Methods
  // "ITAE"   = Integral Time-weighted Absolute Error (smoother)
  // "ZN"     == Ziegler Nichols (faster response, more aggresive)
  std::string method = "ITAE";

  //Safety
  double maxRunTimeMs = 3000;
  double maxDistanceIn = 48;
};

class AutoPIDTuner {
  public:
    //constuctor
    AutoPIDTuner(Drive& drive, tunerConfig config = tunerConfig());

    //Main API
    PIDGains tuneDrive();

    //accessors
    const stepResponseData& getLastStepData() const {
      return _lastData;
    }
    const PIDGains& getLastGains() const {
      return _lastGains;
    }
  private:
    Drive& _drive;
    tunerConfig _cfg;

    stepResponseData _lastData;
    PIDGains _lastGains;

    stepResponseData runStepTest(double voltage);
    stepResponseData averageStepData(const std::vector<stepResponseData>&);
    void analyzeResponse(stepResponseData& data);
    PIDGains calculateGainsITAE(const stepResponseData& data);
    PIDGains calculateGainsZN(const stepResponseData& data);
    void printGains(const PIDGains& gains, const stepResponseData& data);
};