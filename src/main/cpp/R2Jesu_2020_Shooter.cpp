/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::R2Jesu_ProcessShooter()
{
  // Control shooter
  double l_mtrPwr = 0.0;

  if (m_OperatorStick.GetRawButton(9))
  {
    l_mtrPwr = -0.30;
  }
  else if (m_OperatorStick.GetRawButton(10))
  {
    l_mtrPwr = -0.42;
  }
  else if (m_OperatorStick.GetRawButton(11))
  {
    l_mtrPwr = -0.75;
  }
  else if (m_OperatorStick.GetRawButton(12))
  {
    l_mtrPwr = -1.00;
  }
  else if (m_OperatorStick.GetRawButton(1))
  {
    l_mtrPwr = 0.0;
  }

  m_ShooterMotorLeft.Set(l_mtrPwr);
  m_ShooterMotorRight.Set(l_mtrPwr);

  // Shoot the ball
  if (m_OperatorStick.GetRawButton(8))
    ballPopper.Set(true);
  else
    ballPopper.Set(false);

#if R2JESU_TURNON_SMARTDASHBOARD
  frc::SmartDashboard::PutNumber("ShooterMtr", l_mtrPwr);
#endif
}
