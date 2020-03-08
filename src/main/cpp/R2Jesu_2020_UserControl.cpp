/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

// =============================
// Functions to define:
// 1.  Reset robot hang system - Needs to be multi-button and something from driver station

// =============================
// Driver Control Stick
//    Y:  Drive Fwd
//    X:  Turn Robot
//   01:  Robot Brake
//   02:  Turbo ??
//   03:  Align to Target : In Work
//   04:
//   05:
//   06:
//   07:
//   08:
//   09:
//   10:  Hang Robot
//   11:  Reset Hang - Interconnect Btn
//   12:
//   13:

// =============================
// Operator Control Stick Game Pad
//   A-01: Rotate Color Wheel 4 times
//   B-02: Ball Popper
//   X-03: Position Color Wheel
//   04:
//   05: Intake Toggle from 0.00 to 0.50
//   06:
//   Back-07: Break Color Wheel
//   Start-08: Reset Hang - Interconnect Btn
//   09:
//   10:
//   LB-05: Run Intake Backwards
//   RB-06:
//   RT-Throttle Axis: Run Shooter - Toggle
//   LT-Z/Twist Axis: Run Intake

void Robot::R2Jesu_ProcessUserControl()
{

  // Testing NavX
  if (m_Drivestick.GetRawButton(5))
    ahrs->ZeroYaw();

  // Run ball intake
  double l_ballintakeMtr = 0.0;
  if (m_OperatorStick.GetZ() > 0.3)
  {
    l_ballintakeMtr = -1.0;
  }
  else if (m_OperatorStick.GetRawButton(5))
  {
    l_ballintakeMtr = 1.0;
  }

#if R2JESU_TURNON_INTAKE
  snowMotor.Set(l_ballintakeMtr);
#endif

  // Run Winch
  double l_winchMtr = 0.0;

  // Note:
  //   + is CCW from battery side
  //   - is CW  from battery side
  if (m_Drivestick.GetRawButton(10))
  {
    l_winchMtr = -0.4;
  }
// Need to define
#if 1
  else if (m_Drivestick.GetRawButton(12))
  {
    l_winchMtr = 0.4;
  }
#endif

#if R2JESU_TURNON_WINCH
  m_winchMotor.Set(l_winchMtr);
#endif

  // Shoot the ball
  R2Jesu_ProcessShooter();

  R2Jesu_ProcessColorWheel();

  frc::SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
  frc::SmartDashboard::PutNumber("IMU_Yaw", ahrs->GetYaw());
  frc::SmartDashboard::PutNumber("IMU_Pitch", ahrs->GetPitch());
  frc::SmartDashboard::PutNumber("IMU_Roll", ahrs->GetRoll());
  frc::SmartDashboard::PutNumber("IMU_CompassHeading", ahrs->GetCompassHeading());
  // SmartDashboard::PutNumber("IMU_Update_Count", ahrs->GetUpdateCount());
  // SmartDashboard::PutNumber("IMU_Byte_Count", ahrs->GetByteCount());
  // SmartDashboard::PutNumber("IMU_Timestamp", ahrs->GetLastSensorTimestamp());

  /* These functions are compatible w/the WPI Gyro Class */
  frc::SmartDashboard::PutNumber("IMU_TotalYaw", ahrs->GetAngle());
  frc::SmartDashboard::PutNumber("IMU_YawRateDPS", ahrs->GetRate());

  frc::SmartDashboard::PutNumber("IMU_Accel_X", ahrs->GetWorldLinearAccelX());
  frc::SmartDashboard::PutNumber("IMU_Accel_Y", ahrs->GetWorldLinearAccelY());
  frc::SmartDashboard::PutBoolean("IMU_IsMoving", ahrs->IsMoving());
  // SmartDashboard::PutNumber("IMU_Temp_C", ahrs->GetTempC());
  // SmartDashboard::PutBoolean("IMU_IsCalibrating", ahrs->IsCalibrating());

  frc::SmartDashboard::PutNumber("Velocity_X", ahrs->GetVelocityX());
  frc::SmartDashboard::PutNumber("Velocity_Y", ahrs->GetVelocityY());
  frc::SmartDashboard::PutNumber("Displacement_X", ahrs->GetDisplacementX());
  frc::SmartDashboard::PutNumber("Displacement_Y", ahrs->GetDisplacementY());

  /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
  /* NOTE:  These values are not normally necessary, but are made available   */
  /* for advanced users.  Before using this data, please consider whether     */
  /* the processed data (see above) will suit your needs.                     */

  frc::SmartDashboard::PutNumber("RawGyro_X", ahrs->GetRawGyroX());
  frc::SmartDashboard::PutNumber("RawGyro_Y", ahrs->GetRawGyroY());
  frc::SmartDashboard::PutNumber("RawGyro_Z", ahrs->GetRawGyroZ());
  frc::SmartDashboard::PutNumber("RawAccel_X", ahrs->GetRawAccelX());
  frc::SmartDashboard::PutNumber("RawAccel_Y", ahrs->GetRawAccelY());
  frc::SmartDashboard::PutNumber("RawAccel_Z", ahrs->GetRawAccelZ());
  frc::SmartDashboard::PutNumber("RawMag_X", ahrs->GetRawMagX());
  frc::SmartDashboard::PutNumber("RawMag_Y", ahrs->GetRawMagY());
  frc::SmartDashboard::PutNumber("RawMag_Z", ahrs->GetRawMagZ());
  // SmartDashboard::PutNumber("IMU_Temp_C", ahrs->GetTempC());
}
