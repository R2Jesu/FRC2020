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
//   11:
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
//   08:
//   09:
//   10:
//   RB-05:
//   LB-06: Run Intake Backwards
//   RT-Throttle Axis: Run Shooter - Toggle
//   LT-Z/Twist Axis: Run Intake

void Robot::R2Jesu_ProcessUserControl()
{

  frc::SmartDashboard::PutNumber("Twist-axis", m_OperatorStick.GetTwist());
  frc::SmartDashboard::PutNumber("Throt-axis", m_OperatorStick.GetThrottle());
  frc::SmartDashboard::PutNumber("Z-axis", m_OperatorStick.GetZ());

  // Run ball intake
  // QUESTION:  Do we need a reverse?
  double l_ballintakeMtr = 0.0;
  if (m_OperatorStick.GetZ() > 0.3)
  {
    l_ballintakeMtr = -1.0;
  }
  else if (m_OperatorStick.GetRawButton(6))
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
    l_winchMtr = 0.2;
  }
// Need to define
#if 0
  else if (m_Drivestick.GetRawButton(12))
  {
    l_winchMtr = -0.2;
  }
#endif

#if R2JESU_TURNON_WINCH
  m_winchMotor.Set(l_winchMtr);
#endif

  // Shoot the ball
  R2Jesu_ProcessShooter();

  R2Jesu_ProcessColorWheel();
}
