// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>

class Robot : public frc::TimedRobot
{

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
    void Robot::AutonomousInit() override {
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

    void Robot::AutonomousPeriodic() override {
        /*
        if (m_autoSelected == kAutoNameCustom) {
            // Custom Auto goes here
        } else {
            // Default Auto goes here
        }
        */
    }

    void Robot::TeleopInit() override {}
    
	void TeleopPeriodic() override
	{

		double leftInput = m_stick.GetRawAxis(1);
		double rightInput = m_stick.GetRawAxis(5);

		m_leftLeadMotor.Set(leftInput);
		m_rightLeadMotor.Set(rightInput);
	}

	void Robot::DisabledInit() override {}

    void Robot::DisabledPeriodic() override {}

    void Robot::TestInit() override {}

    void Robot::TestPeriodic() override {}

    void Robot::SimulationInit() override {}

    void Robot::SimulationPeriodic() override {}
	

	private:
		//Joysticks
		frc::Joystick m_stick{0};
		// frc::PWMSparkMax m_motor{0};

		// Side wheels
		static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
		rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
		rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
		rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
		rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};

		
		frc::DoubleSolenoid m_doubleSolenoid{frc::PneumaticsModuleType::CTREPCM, 1,
										2};
};

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
