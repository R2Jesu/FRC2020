/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

  Robot::Robot() {
    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
  }

  void Robot::AutonomousInit() 
   {
    m_timer.Reset();
    m_timer.Start();
  }

  void Robot::AutonomousPeriodic() 
   {
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(-0.5, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }

  void Robot::TeleopInit()  {}

  void Robot::TeleopPeriodic() 
   {
    // Drive with arcade style (use right stick)
    m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());

    // Run launch motors when joystick trigger is pulled
    if (m_stick.GetRawButton(1))
    { launchMotorLeft.Set(1);
      launchMotorRight.Set(-1);
    }
    else
    {
      launchMotorLeft.Set(0);
      launchMotorRight.Set(0);
    }
    
  }

  void Robot::TestPeriodic()  {}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
