package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PivotSubsystem extends SubsystemBase{
    private SparkMax m_pivotMotor;
    private SparkMax m_intakeMotor;
   // private AbsoluteEncoder m_pivotEncoder = m_pivotMotor

    public PivotSubsystem(){

        this.m_intakeMotor = new SparkMax(Constants.DriveConstants.m_intakeMotorID, MotorType.kBrushless);
        this.m_pivotMotor = new SparkMax(Constants.DriveConstants.m_pivotMotorID, MotorType.kBrushless);

    }

    public void IntakeIn(){
        m_intakeMotor.set(.5);

    }

    public void IntakeOut(){
        m_intakeMotor.set(-.5);

    }

    public void IntakeStop(){
        m_intakeMotor.set(0);
    }

    public void PivotUp(){
        m_pivotMotor.set(0.2); 
    }
    public void PivotDown(){
        m_pivotMotor.set(-0.2); 
    }
    public void PivotStop(){
        m_pivotMotor.set(0);
    }

    public void ElevatorTestValue(){
        // m_elevatorEncoder.setMinRate(1);;
        // System.out.println(m_elevatorEncoder.getDistance()); 
    }
}
