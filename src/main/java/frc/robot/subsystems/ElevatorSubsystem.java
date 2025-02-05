package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder; 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
//create elevator motors 
private SparkMax m_elevatorFollower; 
private SparkMax m_elevatorMotor;

private SparkMaxConfig followerconfig;
private SparkMaxConfig elevatorconfig;

private PIDController elevatorPID;
private Encoder elevatorEncoder;
 
public ElevatorSubsystem() {

    //value for bottom of elevator 
    this.m_elevatorMotor = new SparkMax (Constants.ElevatorConstants.kElevatorMotorCanId, MotorType.kBrushless); 
    this.m_elevatorFollower = new SparkMax(Constants.ElevatorConstants.kElevatorFollowerCanId,MotorType.kBrushless ); 

    elevatorEncoder = new Encoder(Constants.ElevatorConstants.KEncoderChannelA, Constants.ElevatorConstants.KEncoderChannelB, false);
    elevatorPID = new PIDController(Constants.ElevatorConstants.KP,Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);
    
    followerconfig = new SparkMaxConfig();
    elevatorconfig = new SparkMaxConfig();
    
    followerconfig.idleMode(IdleMode.kBrake);
    elevatorconfig.idleMode(IdleMode.kBrake);
    
    elevatorconfig.inverted(false);
    followerconfig.inverted(true);

    m_elevatorMotor.configure(elevatorconfig, null, null);
    m_elevatorFollower.configure(followerconfig, null, null);
}

public void ElevatorUp(){
    this.m_elevatorMotor.set(0.2);
}

public void ElevatorDown(){
    this.m_elevatorMotor.set(-0.2); 
}
}
