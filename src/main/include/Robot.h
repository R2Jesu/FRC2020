/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/spark.h>

class Robot : public frc::TimedRobot {
 public:

  // Consturctor
  Robot();

  // =================================================
  //  Top Level Robot Functions
  // =================================================

  void AutonomousInit();

  void AutonomousPeriodic();

  void TeleopInit();

  void TeleopPeriodic();

  void TestPeriodic();

 private:

  // =================================================
  //  Subsystem Control Functions
  // =================================================
  
  //  Main Robot Drive
  void R2Jesu_ProcessDrive(void);

  //  Main User Control functions.
  //  This may need to be split out.
  void R2Jesu_ProcessUserControl(void);

  // Process the Color Wheel Control
  void R2Jesu_ProcessColorWheel(void);

  // Robot drive system
  frc::PWMVictorSPX m_left{1};
  frc::PWMVictorSPX m_right{0};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::Joystick m_stick{0};
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
  frc::Spark launchMotorLeft{9}; 
  frc::Spark launchMotorRight{8}; 
};


