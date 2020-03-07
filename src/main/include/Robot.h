/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// WPI Libs
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Motor Controllers
#include <frc/spark.h>
#include <frc/PWMVictorSPX.h>
#include "rev/SparkMax.h"
#include <rev/CANSparkMax.h>

// Color Sensor
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/NidecBrushless.h>
#include "frc/DriverStation.h"

//  Nav Sensor
#include <ctre/Phoenix.h>

// Pneumatics
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

#define R2JESU_TURNON_SMARTDASHBOARD 1

class Robot : public frc::TimedRobot
{
public:
  // Consturctor
  Robot();

  // =================================================
  //  Top Level Robot Functions
  // =================================================

  void AutonomousInit();

  void AutonomousPeriodic();

  void TeleopInit();

  void TeleopPeriodic();

  void TestPeriodic();

private:
  // =================================================
  //  Subsystem Control Functions
  // =================================================

  //  Main Robot Drive
  void R2Jesu_ProcessDrive(void);

  //  Main User Control functions.
  //  This may need to be split out.
  void R2Jesu_ProcessUserControl(void);

  // Shooter
  void R2Jesu_ProcessShooter(void);

  // Process the Color Wheel Control
  void R2Jesu_ProcessColorWheel(void);
  void R2Jesu_CheckGameTargetColor(void);
  frc::Color R2Jesu_ReadColorWheel(void);

  // =================================================
  //  Class Objects
  // =================================================

  // User Control
  frc::Joystick m_Drivestick{0};
  frc::Joystick m_OperatorStick{0};

  // Robot drive system
  frc::PWMVictorSPX m_leftMotor{0};  // Second motor wired to Y PWM Cablec
  frc::PWMVictorSPX m_rightMotor{1}; // Second motor wired to Y PWM Cable
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  // Encoders
  //frc::Encoder m_encL{7, 8, false, frc::Encoder::k4X};
  //frc::Encoder m_encR{4, 5, false, frc::Encoder::k4X};

  // Because motor control cmds may come from other parts of the code use these
  // objects to control the final motor control
  double m_drvL;
  double m_drvR;

  // Ball Intake Subsystem
  WPI_TalonSRX snowMotor{2};

  // Ball Shooter Subsystem - Pneumatics & Motor
  rev::CANSparkMax m_ShooterMotorLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_ShooterMotorRight{3, rev::CANSparkMax::MotorType::kBrushless};
  frc::Compressor compressorObject;
  frc::Solenoid ballPopper{0};

  // Color Wheel Subsystem

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  static constexpr frc::Color kBlueTarget = frc::Color(0.125, 0.421, 0.454);
  static constexpr frc::Color kGreenTarget = frc::Color(0.194, 0.582, 0.223);
  static constexpr frc::Color kRedTarget = frc::Color(0.483, 0.387, 0.13);
  static constexpr frc::Color kYellowTarget = frc::Color(0.312, 0.564, 0.124);
  static constexpr frc::Color nun = frc::Color(0, 0, 0);
  static constexpr frc::Color Default = frc::Color(1, 1, 1);
  double ColorConfidence = 0.0;

  std::string gameData;
  frc::Color gameColor = nun;
  double NidecValue = 0.25;
  frc::NidecBrushless ColorWheelmotor = frc::NidecBrushless(3, 0);

  // Winch Subsystem

  // Support Objects
  frc::Timer m_timer;

  // Debug & feedback
  frc::LiveWindow &m_lw = *frc::LiveWindow::GetInstance();
};