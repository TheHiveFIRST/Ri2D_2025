// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // The motors on the left side of the drive.
  private final CANSparkMax m_leftMotor = new CANSparkMax(5, MotorType.kBrushed);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushed);

  // The motors on the right side of the drive.
  private final CANSparkMax m_rightMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_shootorMotor = new CANSparkMax(1, MotorType.kBrushed);
  
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  private final Joystick m_stick = new Joystick(0);

  public Robot() {
    
    m_rightFollower.follow(m_rightMotor);
    m_leftFollower.follow(m_leftMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightFollower.setInverted(true);

   // SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    //SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
   
    m_robotDrive.arcadeDrive(m_stick.getRawAxis(1), m_stick.getRawAxis(0));
    

    if (m_stick.getRawButtonPressed(1)) {
      m_shootorMotor.set(0.4);
    }

    if (m_stick.getRawButton(2)){
      m_shootorMotor.set(-0.4);
    }

    if (m_stick.getRawButtonReleased(1)){

      m_shootorMotor.set(0);
    }

    if (m_stick.getRawButtonReleased(2)){
      m_shootorMotor.set(0);
    }
    


    }
  
  }

