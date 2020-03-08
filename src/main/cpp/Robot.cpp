/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

double Robot::turning;

void Robot::RobotInit()
{
  // Init Timer
  m_timer.Start();

  // Init Robot Drive
  m_robotDrive.SetExpiration(0.1);
  m_drvL = 0.0;
  m_drvR = 0.0;

// Init Shooter Drive
#if R2JESU_TURNON_SHOOTER
  m_ShooterMotorLeft.RestoreFactoryDefaults();
  m_ShooterMotorRight.RestoreFactoryDefaults();
  //  Invert the right motor
  m_ShooterMotorRight.SetInverted(true);
#endif

// Init Winch Drive
#if R2JESU_TURNON_WINCH
  m_winchMotor.Set(0.0);
#endif

// Init Intake Drive
#if R2JESU_TURNON_INTAKE
  snowMotor.Set(0.0);
#endif

  //  Init Color Sensor
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);
  m_colorMatcher.AddColorMatch(Default);

  // Run Pneumatics
#if R2JESU_TURNON_PNEUMATICS
  // Set Compressor Object for automatic closed loop control
  compressorObject.SetClosedLoopControl(true);
  // Set Solenoids to iniital stat
  ballPopper.Set(false);
#endif

// NavX Sensor
#if R2JESU_TURNON_NAV
  ahrs = new AHRS(frc::SPI::Port::kMXP);
  ahrs->ZeroYaw();
#endif

  // Vision & Camera Init

  // Drive USB Camera - 1
  cs::UsbCamera drvCamera = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  drvCamera.SetResolution(320, 240);
  drvCamera.SetFPS(15);

// Vision Processing Camera - 0
#if R2JESU_TURNON_VISION
  std::thread visionThread(VisionThread);
  visionThread.detach();
#endif
}

void Robot::TeleopInit()
{
  gameColor = nun;
}

void Robot::TeleopPeriodic()
{

  // Set the target color
  R2Jesu_CheckGameTargetColor();

  // Process user control before drive control.
  R2Jesu_ProcessUserControl();

  // Note this needs to come last to merge other drive motor request.
  R2Jesu_ProcessDrive();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
