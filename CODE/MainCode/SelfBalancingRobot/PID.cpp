#include <PID.h>

PID::PID(double *in,double *out, double *set, double P, double I, double D, bool dir, double ltime)
{
  input = in;
  output = out;
  setpoint = set;
  Kp = P;
  Ki = I;
  Kd = D;
  direction = dir;
  PID::setOutputLimits(-500,500);
  PID::setControllerDirection(dir);
  PID::setTunings(P,I,D);
  integral = 0;
  lastTime = ltime;
  lastError = 0;
  deadMin = 0;
  deadMax = 0;
  autoSetpoint = 0;
}

void PID::setOutputLimits(double low, double high)
{
  outMin = low;
  outMax = high;
}

void PID::setControllerDirection(bool dir)
{
  direction = dir;
  PID::setTunings(Kp,Ki,Kd);
}

void PID::setDeadzone(double dmin,double dmax)
{
  deadMin = dmin;
  deadMax = dmax;
}

void PID::setTunings(double P,double I,double D)
{
  if(direction)
  {
    Kp = P;
    Ki = I;
    Kd = D;
  }
  else
  {
    Kp = 0 - P;
    Ki = 0 - I;
    Kd = 0 - D;
  }
}

void PID::compute(double ptime)
{
  double dt = (ptime - lastTime);
  double error = (*input - *setpoint - autoSetpoint);
  double de = (error - lastError);
  double out;
  integral += Ki*error*dt;

  out = Kp*error + integral + Kd*(de/dt);

  if(out >= deadMin && out <= deadMax)
  {
    out = 0;
  }

  if(out > outMax)
  {
    out = outMax;
  }
  if(out < outMin)
  {
    out = outMin;
  }

  *output = out;

  lastTime = ptime;
  lastError = error;
}

double PID::getKp()
{
  return Kp;
}

double PID::getKi()
{
  return Ki;
}

double PID::getKd()
{
  return Kd;
}

bool PID::getDirection()
{
  return direction;
}

void PID::updateTime(double ltime)
{
  lastTime = ltime;
}
























