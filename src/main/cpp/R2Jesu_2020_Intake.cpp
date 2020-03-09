/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::R2Jesu_ProcessIntake()
{
  
  // Run ball intake
  double l_ballintakeMtr = 0.0;
  if ((m_OperatorStick.GetZ() > 0.3) && ballCupLimit.Get()) 
  {
    /*snowMotor.Set(-1.0);
    m_timer.Reset();
    m_timer.Start();
    
    while ((m_timer.Get()<0.1) && (m_OperatorStick.GetZ() > 0.3) && (ballCupLimit.Get()))
    {
        continue;
    }
    while ((m_OperatorStick.GetZ() > 0.3) && (ballCupLimit.Get()) && (lowerLimit.Get()))
    {
        continue;
    }
    while ((m_OperatorStick.GetZ() > 0.3))
    {
        continue;
    }
  snowMotor.Set(0); */
    l_ballintakeMtr = -1.0;

  }
  else if (m_OperatorStick.GetRawButton(5))
  {
    l_ballintakeMtr = 1.0;
  }

#if R2JESU_TURNON_INTAKE
  snowMotor.Set(l_ballintakeMtr);
#endif


}
