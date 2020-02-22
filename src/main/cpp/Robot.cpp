/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

  Robot::Robot() 
  {
    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
  }

  void Robot::TeleopInit()  {}

  void Robot::TeleopPeriodic() 
  {

    // Process user control before drive control.  We may want to switch
     R2Jesu_ProcessUserControl();
     R2Jesu_ProcessDrive();
    
  }

  void Robot::TestPeriodic()  {}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
