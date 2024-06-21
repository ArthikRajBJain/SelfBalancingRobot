#ifndef PID_LIB
#define PID_LIB


class PID
{
  public:
    PID(double*,double*,double*,double,double,double,bool,double);
    void compute(double);
    void setOutputLimits(double,double);
    void setTunings(double,double,double);
    void setControllerDirection(bool);
    void setDeadzone(double,double);
    void updateTime(double);
    double getKp();
    double getKi();
    double getKd();
    bool getDirection();
    double integral;
    double autoSetpoint, autoSetpointGain;
  private:
    double Kp,Ki,Kd;
    double *input, *output, *setpoint;
    double lastError, lastTime;
    bool direction;
    double outMin, outMax;
    double deadMin,deadMax;
};

#endif
