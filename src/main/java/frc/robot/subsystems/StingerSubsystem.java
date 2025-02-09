package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
//import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class StingerSubsystem extends SubsystemBase {
    // Variables
    private SparkMax m_intakeMotor; 
    private SparkMax m_pivotMotor;
    private AbsoluteEncoder m_absencoder;
    private PIDController m_pivotPID;

    // Constructors
    public StingerSubsystem() {
        m_intakeMotor = new SparkMax(Constants.PivotConstants.kPintakeMotorId, MotorType.kBrushless);
        m_pivotMotor = new SparkMax(Constants.PivotConstants.kPivotMotorId, MotorType.kBrushless);
        m_absencoder =  m_pivotMotor.getAbsoluteEncoder();
        m_pivotPID = new PIDController(Constants.PivotConstants.pivotKP,Constants.PivotConstants.pivotKI, Constants.PivotConstants.pivotKD);
    }

    // Methods
    public void setIntakePower(double intakePower){
        m_intakeMotor.set(intakePower);
    }

    public void setPivotPower(double pivotPower){
        m_pivotMotor.set(pivotPower);
    }

    public void goToHeheHaha(int hahahhehe) {
        double output = m_pivotPID.calculate(m_absencoder.getPosition(), hahahhehe);
        m_pivotMotor.set(output);
    }
}