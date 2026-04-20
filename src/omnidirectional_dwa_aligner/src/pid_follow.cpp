#include "pid_follow.h"

pid_follow::pid_follow(double _kp, double _ki, double _kd, double _speedLimit, double _forwardDistance)
: speedLimit(_speedLimit), forwardDistance(_forwardDistance), gyro(0.0)
{
  pid_params[0] = _kp; pid_params[1] = _ki; pid_params[2] = _kd;
  pid_params[3] = _speedLimit; pid_params[4] = -_speedLimit; pid_params[5] = 0.05; pid_params[6] = _forwardDistance;
  Init();
}

pid_follow::pid_follow() : pid_follow(1.0, 0.0, 0.0, 1.0, 0.1) {}

pid_follow::~pid_follow() = default;

void pid_follow::Init()
{
  auto reset_pid = [](PIDFloatDefine* p) {
    p->P = 0.0; p->I = 0.0; p->D = 0.0;
    p->SetPoint = 0.0; p->feedPoint = 0.0; p->OutPoint = 0.0;
    p->OutMax = 1.0; p->OutMin = -1.0; p->DiffError = 0.0; p->IntError = 0.0;
    p->IntLimt = 1.0; p->Intdt = 0.05; p->NowError[0] = p->NowError[1] = p->NowError[2] = 0.0;
    p->LastfeedPoint = 0.0;
  };
  reset_pid(&x_vel_pid);
  reset_pid(&y_vel_pid);
  reset_pid(&gyro_pid);
}

void pid_follow::SetPIDParameter(PIDFloatDefine *p_pid, double *A1)
{
  if (!p_pid || !A1) return;
  p_pid->P = A1[0];
  p_pid->I = A1[1];
  p_pid->D = A1[2];
  p_pid->OutMax = A1[3];
  p_pid->OutMin = A1[4];
  p_pid->Intdt = A1[5];
  p_pid->IntLimt = std::max(std::fabs(p_pid->OutMax), std::fabs(p_pid->OutMin));
}

void pid_follow::PIDFloatPositionCal(PIDFloatDefine *p_pid)
{
  if (!p_pid) return;
  p_pid->NowError[2] = p_pid->NowError[1];
  p_pid->NowError[1] = p_pid->NowError[0];
  p_pid->NowError[0] = p_pid->SetPoint - p_pid->feedPoint;

  p_pid->IntError += p_pid->NowError[0] * p_pid->Intdt;
  p_pid->IntError = LIMIT(p_pid->IntError, -p_pid->IntLimt, p_pid->IntLimt);
  p_pid->DiffError = (p_pid->NowError[0] - p_pid->NowError[1]) / std::max(1e-6, p_pid->Intdt);

  p_pid->OutPoint = p_pid->P * p_pid->NowError[0] + p_pid->I * p_pid->IntError + p_pid->D * p_pid->DiffError;
  p_pid->OutPoint = LIMIT(p_pid->OutPoint, p_pid->OutMin, p_pid->OutMax);
  p_pid->LastfeedPoint = p_pid->feedPoint;
}

Eigen::Vector2d pid_follow::speedOutput(const Eigen::Vector2d _startTrace,
                                        const Eigen::Vector2d _endTrace,
                                        const vector<Eigen::Vector2d> _pathTrace)
{
  (void)_pathTrace;
  Eigen::Vector2d err = _endTrace - _startTrace;
  const double norm = err.norm();
  if (norm > 1e-6 && norm > speedLimit) {
    err *= (speedLimit / norm);
  }
  return err;
}
