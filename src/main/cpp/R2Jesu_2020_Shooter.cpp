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
  double shooterPower = (currentDistance * .0003007) + .3472;
  frc::SmartDashboard::PutNumber("current distance 2", currentDistance);
  if (shooterPower < .39){
    shooterPower = .39;
  }
  if (currentDistance == 0){
    shooterPower = .75;
  }
  if (m_OperatorStick.GetThrottle() > 0.3)
  {
    l_mtrPwr = -shooterPower;
  }
frc::SmartDashboard::PutNumber("shooter power", shooterPower);  
#if 0 // Test Code
  // Rest is test Code 
  else if (m_TestStick.GetRawButton(9))
  {
    l_mtrPwr = -0.30;
  }
  else if (m_TestStick.GetRawButton(10))
  {
    l_mtrPwr = -0.42;
  }
  else if (m_TestStick.GetRawButton(11))
  {
    l_mtrPwr = -0.75;
  }
  else if (m_TestStick.GetRawButton(12))
  {
    l_mtrPwr = -1.00;
  }

#endif

#if R2JESU_TURNON_SHOOTER
  m_ShooterMotorLeft.Set(l_mtrPwr);
  m_ShooterMotorRight.Set(l_mtrPwr);
#endif

// Shoot the ball
#if R2JESU_TURNON_PNEUMATICS

  if (m_OperatorStick.GetRawButton(2))
    ballPopper.Set(true);
  else
    ballPopper.Set(false);

#endif

#if R2JESU_TURNON_SMARTDASHBOARD
  frc::SmartDashboard::PutNumber("ShooterMtr", l_mtrPwr);
#endif
}
