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
#include <frc/drive/DifferentialDrive.h>
#include <frc/Encoder.h>

// Live Window and SmartDashboard
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

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

// NavX
#include "AHRS.h"

// Pneumatics
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

// Camera

#include <thread>
#include <cameraserver/CameraServer.h>
//#include <frc/TimedRobot.h>
//#include <wpi/raw_ostream.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <GripPipeline.h>
#include <cmath>

//Limit Switches
#include <frc/DigitalInput.h>
#include <frc/Timer.h>

#define R2JESU_TURNON_SMARTDASHBOARD 1

// Control robot config for either Fin (1) or Rex (0)
#define R2JESU_FIN_CONFIG 1

#if R2JESU_FIN_CONFIG
#define R2JESU_TURNON_PNEUMATICS 1
#define R2JESU_TURNON_ENCODER 1
#define R2JESU_TURNON_INTAKE 1
#define R2JESU_TURNON_SHOOTER 1
#define R2JESU_TURNON_WINCH 1
#define R2JESU_TURNON_VISION 1
#define R2JESU_TURNON_NAV 1
#else
#define R2JESU_TURNON_PNEUMATICS 0
#define R2JESU_TURNON_ENCODER 0
#define R2JESU_TURNON_INTAKE 0
#define R2JESU_TURNON_SHOOTER 0
#define R2JESU_TURNON_WINCH 0
#define R2JESU_TURNON_VISION 1
#define R2JESU_TURNON_NAV 1
#endif

class Robot : public frc::TimedRobot
{
public:
  // Consturctor
  //Robot();

  // =================================================
  //  Top Level Robot Functions
  // =================================================

  void RobotInit();

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
  void R2Jesu_ProcessDrive(double p_LimitFactor = 1.0);

  //  Main User Control functions.
  //  This may need to be split out.
  void R2Jesu_ProcessUserControl(void);

  // Shooter
  void R2Jesu_ProcessShooter(void);

  // Process the Color Wheel Control
  void R2Jesu_ProcessColorWheel(void);
  void R2Jesu_CheckGameTargetColor(void);
  frc::Color R2Jesu_ReadColorWheel(void);

  //Intake Processing
  void R2Jesu_ProcessIntake(void);

  // Vision Processing
  static void VisionThread(void);

  // =================================================
  //  Class Objects
  // =================================================

  // User Control
  frc::Joystick m_Drivestick{0};
  frc::Joystick m_OperatorStick{1};
  frc::Joystick m_TestStick{5};

  // Robot drive system
  frc::PWMVictorSPX m_leftMotor{0};  // Second motor wired to Y PWM Cablec
  frc::PWMVictorSPX m_rightMotor{1}; // Second motor wired to Y PWM Cable
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

// Encoders
#if R2JESU_TURNON_ENCODER
  frc::Encoder m_encL{7, 8, false, frc::Encoder::k4X};
  frc::Encoder m_encR{4, 5, false, frc::Encoder::k4X};
#endif

  // Because motor control cmds may come from other parts of the code use these
  // objects to control the final motor control
  double m_drvL;
  double m_drvR;

  // Ball Intake Subsystem
  WPI_TalonSRX snowMotor{2};

// Ball Shooter Subsystem - Pneumatics & Motor
#if R2JESU_TURNON_SHOOTER
  rev::CANSparkMax m_ShooterMotorLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_ShooterMotorRight{3, rev::CANSparkMax::MotorType::kBrushless};
  frc::DigitalInput lowerLimit{1};
  frc::DigitalInput ballCupLimit{2};
#endif

#if R2JESU_TURNON_PNEUMATICS
  frc::Compressor compressorObject;
  frc::Solenoid ballPopper{0};
  frc::Solenoid colorArm{1};
#endif


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
  const double NidecValue = 0.35;
  frc::NidecBrushless ColorWheelmotor = frc::NidecBrushless(3, 0);

  // Winch Subsystem
  WPI_TalonSRX m_winchMotor{4};

  // Vision System
  static double turning;

// NavX
#if R2JESU_TURNON_NAV
  AHRS *ahrs;
#endif

  // Support Objects
  frc::Timer m_timer;

  //  Driver station Control
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // Debug & feedback
  frc::LiveWindow &m_lw = *frc::LiveWindow::GetInstance();
};