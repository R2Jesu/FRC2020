/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

/**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */

void Robot::R2Jesu_ProcessColorWheel()
{
  if (m_Drivestick.GetRawButton(8))
  { //red button
    //run motor until game color is seen twice

    int colorCounter = 0;
    ColorWheelmotor.Set(NidecValue);
    while (colorCounter < 1 && !(m_Drivestick.GetRawButton(1)))
    {
      if (gameColor == R2Jesu_ReadColorWheel())
      {
        colorCounter++;
      }
      // Allow for operator driving
      R2Jesu_ProcessDrive();
    }

    ColorWheelmotor.Set(0.0);
  }

  if (m_Drivestick.GetRawButton(7))
  {
    //run motor until game color is seen 9 times

    int colorCount = 0;
    int colorCount2 = 0;
    ColorWheelmotor.Set(NidecValue);

    /**
     * Run the color match algorithm on our detected color
     */
    frc::Color startingColor = R2Jesu_ReadColorWheel();

    while (colorCount < 9 && !(m_Drivestick.GetRawButton(1)))
    {
      if (startingColor == R2Jesu_ReadColorWheel())
      {
        if (colorCount < 8)
        {
          colorCount2 = 0;
          while (colorCount2 < 1 && !(m_Drivestick.GetRawButton(1)))
          {
            if (!(startingColor == R2Jesu_ReadColorWheel()))
            {
              colorCount2++;
            }
          }
        }
        colorCount++;
      }
      // Allow for operator driving
      R2Jesu_ProcessDrive();
    }
    ColorWheelmotor.Set(0.0);
  }
}

void Robot::R2Jesu_CheckGameTargetColor()
{
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if ((gameData.length() > 0) && (gameColor == nun))
  {
    switch (gameData[0])
    {
    case 'B':
      gameColor = kRedTarget;
      break;
    case 'G':
      gameColor = kYellowTarget;
      break;
    case 'R':
      gameColor = kBlueTarget;
      break;
    case 'Y':
      gameColor = kGreenTarget;
      break;
    default:
      gameColor = Default;
      break;
    }
  }
#if R2JESU_TURNON_SMARTDASHBOARD
  frc::SmartDashboard::PutString("gameData", gameData);
  frc::SmartDashboard::PutNumber("gameColor", gameColor.red);
#endif
}

frc::Color Robot::R2Jesu_ReadColorWheel()
{

  std::string colorString;

  frc::Color detectedColor = m_colorSensor.GetColor();

  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, ColorConfidence);

  if (matchedColor == kBlueTarget)
  {
    colorString = "Blue";
  }
  else if (matchedColor == kRedTarget)
  {
    colorString = "Red";
  }
  else if (matchedColor == kGreenTarget)
  {
    colorString = "Green";
  }
  else if (matchedColor == kYellowTarget)
  {
    colorString = "Yellow";
  }
  else
  {
    colorString = "Unknown";
  }

#if R2JESU_TURNON_SMARTDASHBOARD
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Confidence", ColorConfidence);
  frc::SmartDashboard::PutString("Detected Color", colorString);
#endif

  return (detectedColor);
}
