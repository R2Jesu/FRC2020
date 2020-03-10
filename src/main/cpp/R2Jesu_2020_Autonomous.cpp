/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

  void Robot::AutonomousInit() 
  {
    m_timer.Reset();
    m_timer.Start();
  }

  void Robot::AutonomousPeriodic() 
  {
    m_ShooterMotorLeft.Set(-.45);
    m_ShooterMotorRight.Set(-.45);
    // Drive for 2 seconds
    if (m_timer.Get() < 5.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(-0.3, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
    if (!(ballCupLimit.Get())) {
      snowMotor.Set(0);
    ballPopper.Set(true);
    } else {
      ballPopper.Set(false);
      snowMotor.Set(-1);
    }
  }

 