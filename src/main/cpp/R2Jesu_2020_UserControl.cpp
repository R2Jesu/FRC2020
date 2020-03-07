/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

// =============================
// Driver Control Stick
//    Y:  Drive Fwd
//    X:  Turn Robot
//   01: 
//   02:
//   03:
//   04:
//   05: 
//   06:
//   07:
//   08: 
//   09: 
//   10: 
//   11: 
//   12: 
//   13: 


// =============================
// Operator Control Stick
//    Y:
//    X:
//   01: Shooter Speed 0.00
//   02:
//   03:
//   04:
//   05: Intake Toggle from 0.00 to 0.50
//   06:
//   07:
//   08: Shooter Ball Popper Toggle
//   09: Shooter Speed 0.30
//   10: Shooter Speed 0.42
//   11: Shooter Speed 0.75
//   12: Shooter Speed 1.00
//   13: Shooter Speed 


void Robot::R2Jesu_ProcessUserControl()
{
  // Run ball intake
  // QUESTION:  Do we need a reverse?
  if (m_OperatorStick.GetRawButton(5))
  {
    snowMotor.Set(-0.5);
  }
  else
  {
    snowMotor.Set(0.0);
  }

  // Shoot the ball
  R2Jesu_ProcessShooter();

  R2Jesu_ProcessColorWheel();

}
