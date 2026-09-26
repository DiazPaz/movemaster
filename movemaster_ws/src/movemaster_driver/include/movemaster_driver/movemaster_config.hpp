// movemaster_config.hpp
//
// Valores por defecto del Movemaster dentro del código.
// El plugin de ros2_control puede sobrescribir cualquiera de ellos desde el
// URDF (<param> de cada <joint>); si un parámetro no aparece, se usa el de aquí.
//
// La arquitectura soporta hasta kMaxJoints = 6 SPARK MAX; la configuración
// actual contempla kJointCount = 3 SPARK MAX, cada uno con su reductor.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "movemaster_driver/gear_transmission.hpp"

namespace movemaster_driver::config
{

/// Máximo de controladores que maneja MoveMasterDriver.
inline constexpr std::size_t kMaxJoints = 6;

/// SPARK MAX actualmente instalados.
inline constexpr std::size_t kJointCount = 3;

inline constexpr std::array<const char *, kJointCount> kJointNames{
  "joint_1", "joint_2", "joint_3"};

/// ID CAN configurado en cada SPARK MAX (REV Hardware Client).
inline constexpr std::array<uint8_t, kJointCount> kCanIds{1, 2, 3};

// =============================================================================
// REDUCTORES — relación de reducción de cada eje
// -----------------------------------------------------------------------------
// gear_ratio = vueltas del motor NEO por cada vuelta de la articulación.
//   Ej.: reductor planetario 100:1          → 100.0
//        dos etapas 5:1 y 4:1 en serie       → GearTransmission::ratioFromStages({5, 4}) = 20.0
//
// TODO(movemaster): sustituir 1.0 por la reducción real de cada eje.
// 1.0 es un valor seguro mientras tanto: con un reductor real la articulación
// se mueve MENOS de lo pedido, nunca más.
// =============================================================================
inline constexpr double kGearRatioJoint1 = 1.0;
inline constexpr double kGearRatioJoint2 = 1.0;
inline constexpr double kGearRatioJoint3 = 1.0;

inline constexpr std::array<double, kJointCount> kGearRatios{
  kGearRatioJoint1, kGearRatioJoint2, kGearRatioJoint3};

/// Sentido de giro motor ↔ articulación.
inline constexpr std::array<bool, kJointCount> kInverted{false, false, false};

/// Posición de la articulación (rad) cuando el encoder del motor marca 0.
inline constexpr std::array<double, kJointCount> kOffsetsRad{0.0, 0.0, 0.0};

/// Límites de posición de la articulación (rad). Se validan en el host.
inline constexpr std::array<double, kJointCount> kMinPositionRad{-kPi, -kPi, -kPi};
inline constexpr std::array<double, kJointCount> kMaxPositionRad{kPi, kPi, kPi};

/// Ganancias PIDF del slot 0 (valores del ejemplo validado en banco: P=1, I=D=F=0).
/// Están en unidades del SPARK (rotaciones del motor), no de la articulación.
inline constexpr int kPidSlot = 0;
inline constexpr double kDefaultP = 1.0;
inline constexpr double kDefaultI = 0.0;
inline constexpr double kDefaultD = 0.0;
inline constexpr double kDefaultF = 0.0;

/// Interfaz SocketCAN y periodo de STATUS_0/STATUS_2 (ms).
inline constexpr const char * kCanInterface = "can0";
inline constexpr int kStatusPeriodMs = 20;

}  // namespace movemaster_driver::config
