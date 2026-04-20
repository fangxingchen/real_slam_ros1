#include "vel_transform.h"

namespace vel_transform
{
void LocalVelocityToGlobal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
  if (!localVelocity || !globalVelocity) return;
  const double c = std::cos(localVelocity->yaw);
  const double s = std::sin(localVelocity->yaw);
  globalVelocity->Vx = c * localVelocity->Vx - s * localVelocity->Vy;
  globalVelocity->Vy = s * localVelocity->Vx + c * localVelocity->Vy;
}

void LocalAcceleraToGlobal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
  if (!localVelocity || !globalVelocity) return;
  const double c = std::cos(localVelocity->yaw);
  const double s = std::sin(localVelocity->yaw);
  globalVelocity->Accx = c * localVelocity->Accx - s * localVelocity->Accy;
  globalVelocity->Accy = s * localVelocity->Accx + c * localVelocity->Accy;
}

void GlobalVelocityToLocal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
  GlobalVelocityToLocalVector(localVelocity, globalVelocity);
}

void GlobalVelocityToLocalVelocity(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
  GlobalVelocityToLocalVector(localVelocity, globalVelocity);
}

void GlobalVelocityToLocalVector(Odom_data_define *localVelocity, Odom_data_define *globalVelocity)
{
  if (!localVelocity || !globalVelocity) return;
  const double c = std::cos(localVelocity->yaw);
  const double s = std::sin(localVelocity->yaw);
  localVelocity->Vx = c * globalVelocity->Vx + s * globalVelocity->Vy;
  localVelocity->Vy = -s * globalVelocity->Vx + c * globalVelocity->Vy;
  localVelocity->theta = std::atan2(localVelocity->Vy, localVelocity->Vx);
}

double calPointLength(Vector2d vector1, Vector2d vector2)
{
  return (vector1 - vector2).norm();
}

double calVectorAngle(Vector2d vector1, Vector2d vector2)
{
  const double denom = std::max(1e-9, vector1.norm() * vector2.norm());
  double c = vector1.dot(vector2) / denom;
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c);
}

Vector2d calUnitvector(Vector2d unitv)
{
  const double n = unitv.norm();
  if (n < 1e-9) return Vector2d::Zero();
  return unitv / n;
}
} // namespace vel_transform
