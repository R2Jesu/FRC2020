/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::R2Jesu_ProcessDrive()
{

#if 0 // Need to update to merge in other motor requests
  m_drvL += m_Drivestick.GetY();
  m_drvR += m_Drivestick.GetX();
#endif

  // Drive with arcade style (use right stick)
  m_robotDrive.ArcadeDrive(m_Drivestick.GetY(), m_Drivestick.GetX());
}
