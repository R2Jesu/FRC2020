/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::R2Jesu_ProcessDrive(double p_LimitFactor)
{
  // Limit the drive motors for certain operations like color wheel
  if (m_Drivestick.GetRawButton(1)) // Break
  {
    m_drvL = 0.0;
    m_drvR = 0.0;
  }
  else
  {
    m_drvL = -m_Drivestick.GetY() * p_LimitFactor;
    m_drvR = m_Drivestick.GetX() * p_LimitFactor;
  }

  // Drive with arcade style (use right stick)
  m_robotDrive.ArcadeDrive(m_drvL, m_drvR);

#if R2JESU_TURNON_SMARTDASHBOARD
  frc::SmartDashboard::PutNumber("DrvL", m_drvL);
  frc::SmartDashboard::PutNumber("DrvR", m_drvR);
  frc::SmartDashboard::PutNumber("DrvLimit", p_LimitFactor);
#endif

  // Testing NavX
  if (m_Drivestick.GetRawButton(8))
    ahrs->ZeroYaw();

 bool rotateToAngle = false;
  if (m_Drivestick.GetRawButton(4))
  {
    turnController->SetSetpoint(0.0f);
    rotateToAngle = true;
  }
  else if (m_Drivestick.GetRawButton(5))
  {
    turnController->SetSetpoint(20.0f);
    rotateToAngle = true;
  }
  else if (m_Drivestick.GetRawButton(6))
  {
    turnController->SetSetpoint(-20.0f);
    rotateToAngle = true;
  }
  else if (m_Drivestick.GetRawButton(3))
  {
    turnController->SetSetpoint(-10.0f);
    rotateToAngle = true;
  }
  double currentRotationRate = 0.0;
  if (rotateToAngle)
  {
    turnController->Enable();
    currentRotationRate = rotateToAngleRate;
  }
  else
  {
    turnController->Disable();
    rotateToAngle = false;
  }

  if (rotateToAngle)
    m_robotDrive.ArcadeDrive(0.0, currentRotationRate, true);

  frc::SmartDashboard::PutNumber("CurrRotRate", currentRotationRate);
  frc::SmartDashboard::PutBoolean("RotateFlag", rotateToAngle);
  frc::SmartDashboard::PutNumber("CurrSetPt", turnController->GetSetpoint());

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
