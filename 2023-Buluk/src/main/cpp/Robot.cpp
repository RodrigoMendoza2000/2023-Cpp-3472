// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/shuffleboard/Shuffleboard.h>

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

    static const int upperCenterMotorID = 3, lowerCenterMotorID = 4;
    frc::PWMVictorSPX m_upperCenterMotor{upperCenterMotorID};
    frc::PWMVictorSPX m_lowerCenterMotor{lowerCenterMotorID};

    static const int firstSolenoidID = 0, secondSolenoidID = 3;
    frc::DoubleSolenoid s_piston{frc::PneumaticsModuleType::CTREPCM, firstSolenoidID, secondSolenoidID};

    static const int ElevatorID = 5;
    rev::CANSparkMax m_elevatorMotor{ElevatorID, rev::CANSparkMax::MotorType::kBrushless};

    static const int articID = 6;
    rev::CANSparkMax m_articMotor{articID, rev::CANSparkMax::MotorType::kBrushless};


public:
    void RobotInit() override{
        //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
        //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
        //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		m_leftLeadMotor.RestoreFactoryDefaults();
		m_rightLeadMotor.RestoreFactoryDefaults();
		m_leftFollowMotor.RestoreFactoryDefaults();
		m_rightFollowMotor.RestoreFactoryDefaults();

        m_leftFollowMotor.Follow(m_leftLeadMotor);
		m_rightFollowMotor.Follow(m_rightLeadMotor);

        m_elevatorMotor.RestoreFactoryDefaults();

        m_articMotor.RestoreFactoryDefaults();

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
        /*
        m_autoSelected = m_chooser.GetSelected();
        // m_autoSelected = SmartDashboard::GetString("Auto Selector",
        //     kAutoNameDefault);
        fmt::print("Auto selected: {}\n", m_autoSelected);

        if (m_autoSelected == kAutoNameCustom) {
            // Custom Auto goes here
        } else {
            // Default Auto goes here
        }
        */
    }

    void AutonomousPeriodic() override {
        /*
        if (m_autoSelected == kAutoNameCustom) {
            // Custom Auto goes here
        } else {
            // Default Auto goes here
        }
        */
    }

    void TeleopInit() override {}
    
	void TeleopPeriodic() override
	{

		double leftInput = chassis_stick.GetRawAxis(1);
		double rightInput = chassis_stick.GetRawAxis(5) * -1;
        double leftTrigger = chassis_stick.GetRawAxis(2);
        double rightTrigger = chassis_stick.GetRawAxis(3);

        bool aButton = mechanism_stick.GetRawButton(1);
        bool bButton = mechanism_stick.GetRawButton(2);


        double leftInputMechanism = mechanism_stick.GetRawAxis(1) * -1;
        double rightInputMechanism = mechanism_stick.GetRawAxis(5);

        frc::SmartDashboard::PutNumber("Joystick Right Input value", rightInput);
        frc::SmartDashboard::PutNumber("Joystick Left Input value", leftInput);
        frc::SmartDashboard::PutBoolean("Joystick Button A", aButton);
        frc::SmartDashboard::PutBoolean("Joystick Button B", bButton);




        if (leftInputMechanism > 0.1 || leftInputMechanism < 0.1){
            m_elevatorMotor.Set(leftInputMechanism);
        }

        if (rightInputMechanism > 0.1 || rightInputMechanism < 0.1){
            m_articMotor.Set(rightInputMechanism * 0.4);
        }
        

		m_leftLeadMotor.Set(leftInput);
		m_rightLeadMotor.Set(rightInput);

        if (leftTrigger > 0.1){
            m_upperCenterMotor.Set(leftTrigger * -1);
            m_lowerCenterMotor.Set(leftTrigger);
        }
        else{
            m_upperCenterMotor.Set(rightTrigger);
            m_lowerCenterMotor.Set(rightTrigger * -1);
        }

        
        if (aButton){
            s_piston.Set(frc::DoubleSolenoid::kForward);
        } else{
            s_piston.Set(frc::DoubleSolenoid::kReverse);
        }
        

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
