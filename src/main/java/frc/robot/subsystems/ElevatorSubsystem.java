package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

public double demand = 0;
private static final double kLifterEncoderDistPerPulse = 2.0 * Math.PI /4096;
 
public ElevatorSubsystem() {

    //value for bottom of elevator 
    this.m_elevatorMotor = new SparkMax (Constants.ElevatorConstants.kElevatorMotorCanId, MotorType.kBrushless); 
    this.m_elevatorFollower = new SparkMax(Constants.ElevatorConstants.kElevatorFollowerCanId,MotorType.kBrushless ); 

    elevatorEncoder = new Encoder(Constants.ElevatorConstants.KEncoderChannelA,Constants.ElevatorConstants.KEncoderChannelB, false);
    elevatorPID = new PIDController(Constants.ElevatorConstants.KP,Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);
    elevatorEncoder.setDistancePerPulse(kLifterEncoderDistPerPulse);
    
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
    demand = Units.inchesToMeters(12);
    double pidOutput = elevatorPID.calculate(elevatorEncoder.getDistance(),demand);

    this.m_elevatorMotor.set(pidOutput);
    this.m_elevatorFollower.set(pidOutput);
}

public void ElevatorDown(){
    demand = Units.inchesToMeters(0);
    double pidOutput = elevatorPID.calculate(elevatorEncoder.getDistance(),demand);

   
    this.m_elevatorMotor.set(pidOutput);
    this.m_elevatorFollower.set(pidOutput); 
}
}
