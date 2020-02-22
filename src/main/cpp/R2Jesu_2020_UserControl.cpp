/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

  void Robot::R2Jesu_ProcessUserControl() 
  {
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

    // Color Wheel processing
    R2Jesu_ProcessColorWheel();

  }

 
 