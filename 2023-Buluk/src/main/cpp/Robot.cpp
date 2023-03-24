// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>
#define _USE_MATH_DEFINES
#include <cmath>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot
{

private:
    //Joysticks
    frc::Joystick chassis_stick{0};
    frc::Joystick mechanism_stick{1};

    // Side wheels
    static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
    rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};

    // Setup differential drive to change between Arcade and Tank
    frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

    // Center wheels which are set in PWM
    static const int upperCenterMotorID = 3, lowerCenterMotorID = 4;
    frc::PWMVictorSPX m_upperCenterMotor{upperCenterMotorID};
    frc::PWMVictorSPX m_lowerCenterMotor{lowerCenterMotorID};

    // Extend garra solenoid
    static const int firstSolenoidID = 0, secondSolenoidID = 3;
    frc::DoubleSolenoid s_piston{frc::PneumaticsModuleType::CTREPCM, firstSolenoidID, secondSolenoidID};

    // Elevator
    static const int ElevatorID = 5;
    rev::CANSparkMax m_elevatorMotor{ElevatorID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder m_encoder = m_elevatorMotor.GetEncoder();

    // Articulation to lower or upper the intake
    static const int articID = 6;
    rev::CANSparkMax m_articMotor{articID, rev::CANSparkMax::MotorType::kBrushless};

    // Time for autonomous period
    frc::Timer m_timer;

    // Variable for it to begin in tank mode and change between tank and arcade
    bool tankDriveConfig{true};

    // To be able to change autonomous
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

public:
    void RobotInit() override{
        // Necessary initialization of CAN Spark MAX
		m_leftLeadMotor.RestoreFactoryDefaults();
		m_rightLeadMotor.RestoreFactoryDefaults();
		m_leftFollowMotor.RestoreFactoryDefaults();
		m_rightFollowMotor.RestoreFactoryDefaults();
        m_elevatorMotor.RestoreFactoryDefaults();
        m_articMotor.RestoreFactoryDefaults();

        m_robotDrive.SetExpiration(100_ms);

        // For the follower motor to follow the power of lead motor without doing a set value
        m_leftFollowMotor.Follow(m_leftLeadMotor);
		m_rightFollowMotor.Follow(m_rightLeadMotor);

        
        // Start the timer for autonomous period
        m_timer.Start();

        //To be able to select between autonomous programs
        m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
        m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
        frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
        

	}

    /**
     * This function is called every 20 ms, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p> This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
    */
	void RobotPeriodic() override {}

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the GetString line to get the
     * auto name from the text box below the Gyro.
     *
     * You can add additional auto modes by adding additional comparisons to the
     * if-else structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    void AutonomousInit() override {
        // Reset the timer so that it begins in autonomous
        m_timer.Restart();

        // Get the selected autonomous in the shuffleboard
        m_autoSelected = m_chooser.GetSelected();
    }

    void AutonomousPeriodic() override {
        // If the autonomous selected is not the default, advance forward
        if (m_autoSelected == kAutoNameCustom) {
            if (m_timer.Get() < 2_s) {
            m_robotDrive.ArcadeDrive(0.0, 0.3, false);
            
            } 
            else {
            m_robotDrive.ArcadeDrive(0.0, 0.0, false);
            }
        // If it's the default, advance backward
        } else {
            if (m_timer.Get() < 2_s) {
            m_robotDrive.ArcadeDrive(0.0, -0.3, false);
            
            } 
            else {
            m_robotDrive.ArcadeDrive(0.0, 0.0, false);
            }
        }

        // Move the elevator until the encoder reaches a certain value
        /*
        if ((m_encoder.GetPosition() / (2 * M_PI)) < 10){
            m_elevatorMotor.Set(0.2);
        } else{
            m_elevatorMotor.Set(0.0);
        }*/

        // Send the data into the shuffleboard from the encoder
        frc::SmartDashboard::PutNumber("Encoder Position", m_encoder.GetPosition() / (2 * M_PI));
        frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoder.GetVelocity());
        
    }

    void TeleopInit() override {
    }
    
	void TeleopPeriodic() override
	{
        // Declaration of buttons and axis from both joysticks
		double chassis_leftYAxis = chassis_stick.GetRawAxis(1);
		double chassis_rightYAxis = chassis_stick.GetRawAxis(5) * -1;
        double chassis_leftXAxis = chassis_stick.GetRawAxis(0);

        double chassis_leftTrigger = chassis_stick.GetRawAxis(2);
        double chassis_rightTrigger = chassis_stick.GetRawAxis(3);
    
        bool chassis_selectButton = chassis_stick.GetRawButton(7);
        bool chassis_startButton = chassis_stick.GetRawButton(8);

        bool mechanism_aButton = mechanism_stick.GetRawButton(1);
        bool mechanism_bButton = mechanism_stick.GetRawButton(2);

        double mechanism_LeftYAxis = mechanism_stick.GetRawAxis(1) * -1;
        double mechanism_RightYAxis = mechanism_stick.GetRawAxis(5);

        // Send the joystick left and right value to the shuffleboard
        frc::SmartDashboard::PutNumber("Joystick chassis left Y Axis", chassis_leftYAxis);
        frc::SmartDashboard::PutNumber("Joystick chassis right Y axis", chassis_rightYAxis);
        frc::SmartDashboard::PutBoolean("Joystick mechanism Button A", mechanism_aButton);
        frc::SmartDashboard::PutBoolean("Joystick mechanism Button B", mechanism_bButton);


        // Move the elevator, the condition is because both axises always have a value
        // even if it's small, so the elevator only moves when significantly moving the axis
        if (mechanism_LeftYAxis > 0.1 || mechanism_LeftYAxis < 0.1){
            m_elevatorMotor.Set(mechanism_LeftYAxis * 0.4);
        }

        if (mechanism_LeftYAxis > 0.1 || mechanism_RightYAxis < 0.1){
            m_articMotor.Set(mechanism_RightYAxis * 0.4);
        }
        
        // If the select button is selected, change to arcade, if start button is selected, change to tank
        if (chassis_selectButton){
            tankDriveConfig = false;
        }
        else if (chassis_startButton){
            tankDriveConfig = true;
        }

        // Change between tank and arcade drive
        if (tankDriveConfig){
            m_robotDrive.TankDrive(chassis_leftYAxis, chassis_rightYAxis, false);
        } else{
            m_robotDrive.ArcadeDrive(-chassis_leftXAxis, -chassis_leftYAxis, false);
        }

        if (chassis_leftTrigger > 0.1){
            m_upperCenterMotor.Set(chassis_leftTrigger * -1);
            m_lowerCenterMotor.Set(chassis_leftTrigger);
        }
        else{
            m_upperCenterMotor.Set(chassis_rightTrigger);
            m_lowerCenterMotor.Set(chassis_rightTrigger * -1);
        }

        
        // If the a button is pressed, forward the piston. When not pressed reverse it.
        if (mechanism_aButton){
            s_piston.Set(frc::DoubleSolenoid::kForward);
        } else{
            s_piston.Set(frc::DoubleSolenoid::kReverse);
        }

        // Encoder position and velocity to dashboard
        frc::SmartDashboard::PutNumber("Encoder Position", m_encoder.GetPosition() / (2 * M_PI));
        frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoder.GetVelocity());
        

	}

	void DisabledInit() override {}

    void DisabledPeriodic() override {}

    void TestInit() override {}

    void TestPeriodic() override {}

    void SimulationInit() override {}

    void SimulationPeriodic() override {}
};

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
