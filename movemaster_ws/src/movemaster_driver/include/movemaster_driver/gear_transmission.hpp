// gear_transmission.hpp
//
// Conversión paramétrica entre la articulación (rad, rad/s) y el motor NEO
// (rotaciones, RPM) a través de un reductor.
//
//   gear_ratio = rotaciones del MOTOR por cada rotación de la ARTICULACIÓN
//                (reductor 100:1 → gear_ratio = 100)
//
//   motor_rot   = dir · (q − offset) · gear_ratio / 2π
//   q           = offset + dir · motor_rot · 2π / gear_ratio
//   motor_rpm   = dir · q̇ · gear_ratio · 60 / 2π
//   q̇           = dir · motor_rpm · 2π / (60 · gear_ratio)
//
// donde dir = −1 si el eje está invertido y offset es la posición de la
// articulación (rad) cuando el encoder del motor marca 0 rotaciones.
//
// El SPARK trabaja siempre en rotaciones/RPM del MOTOR (factores de conversión
// = 1, como en teach_pendant_backend.py); la conversión se hace en el host, así
// las ganancias PID validadas en el banco siguen siendo válidas.

#pragma once

#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace movemaster_driver
{

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

/// Conversiones sin reductor.
inline constexpr double radiansToRotations(double radians) {return radians / kTwoPi;}
inline constexpr double rotationsToRadians(double rotations) {return rotations * kTwoPi;}
inline constexpr double radPerSecToRpm(double rad_per_s) {return rad_per_s * 60.0 / kTwoPi;}
inline constexpr double rpmToRadPerSec(double rpm) {return rpm * kTwoPi / 60.0;}

struct GearTransmission
{
  /// Rotaciones del motor por rotación de la articulación (> 0).
  double gear_ratio{108.0};
  /// Invierte el sentido motor ↔ articulación.
  bool inverted{false};
  /// Posición de la articulación (rad) cuando el encoder del motor marca 0.
  double offset_rad{0.0};

  /// Relación total de un tren de engranes / etapas en serie: {5, 4, 5} → 100.
  static double ratioFromStages(std::initializer_list<double> stages)
  {
    double ratio = 1.0;
    for (double stage : stages) {
      if (!(stage > 0.0) || !std::isfinite(stage)) {
        throw std::invalid_argument("Cada etapa del reductor debe ser finita y > 0");
      }
      ratio *= stage;
    }
    return ratio;
  }

  void validate() const
  {
    if (!(gear_ratio > 0.0) || !std::isfinite(gear_ratio)) {
      throw std::invalid_argument("gear_ratio debe ser finito y > 0");
    }
    if (!std::isfinite(offset_rad)) {
      throw std::invalid_argument("offset_rad debe ser finito");
    }
  }

  double direction() const {return inverted ? -1.0 : 1.0;}

  /// rad de la articulación → rotaciones del motor.
  double jointToMotorRotations(double joint_rad) const
  {
    return direction() * (joint_rad - offset_rad) * gear_ratio / kTwoPi;
  }

  /// rotaciones del motor → rad de la articulación.
  double motorRotationsToJoint(double motor_rotations) const
  {
    return offset_rad + direction() * motor_rotations * kTwoPi / gear_ratio;
  }

  /// rad/s de la articulación → RPM del motor.
  double jointVelocityToMotorRpm(double joint_rad_per_s) const
  {
    return direction() * joint_rad_per_s * gear_ratio * 60.0 / kTwoPi;
  }

  /// RPM del motor → rad/s de la articulación.
  double motorRpmToJointVelocity(double motor_rpm) const
  {
    return direction() * motor_rpm * kTwoPi / (60.0 * gear_ratio);
  }
};

}  // namespace movemaster_driver
