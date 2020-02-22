/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

  void Robot::R2Jesu_ProcessDrive() 
  {
     // Drive with arcade style (use right stick)
    m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());

  }



 