package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public SparkMax leftElevator;
    public SparkMax rightElevator;

    public SparkMaxConfig leftConfig;
    public SparkMaxConfig rightConfig;
  
    public Encoder elevatorEncoder;

    private PIDController elevatorPID;

    private final double bottomPosition;
    private final double topPosition;
    private final double middlePosition;

    public enum ElevatorPosition {
        BOTTOM, MIDDLE, TOP
    }

    private enum ElevatorState {
        IDLE, MOVING_TO_BOTTOM, MOVING_TO_MIDDLE, MOVING_TO_TOP
    }

    private ElevatorState currentState = ElevatorState.IDLE;
    @SuppressWarnings("unused")
    private double targetPosition;

    public ElevatorSubsystem() {
        leftElevator = new SparkMax(Constants.ElevatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);
        rightElevator = new SparkMax(Constants.ElevatorConstants.kElevatorMotorCanId, MotorType.kBrushless);

        elevatorEncoder = new Encoder(ElevatorConstants.KEncoderChannelA, ElevatorConstants.KEncoderChannelB);
        elevatorEncoder.setDistancePerPulse(2*Math.PI*2/4096);

        elevatorPID = new PIDController(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);

        bottomPosition = Constants.ElevatorConstants.BOTTOM_POSITION;
        middlePosition = Constants.ElevatorConstants.MIDDLE_POSITION;
        topPosition = Constants.ElevatorConstants.TOP_POSITION;
      
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        leftElevator.configure(leftConfig, null, null);
        rightElevator.configure(rightConfig, null, null);
    }

    public void runElevatorUp() {

        leftElevator.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
        rightElevator.set(Constants.ElevatorConstants.ELEVATOR_SPEED);

        System.out.println(elevatorEncoder.getDistance());
        SmartDashboard.putNumber("encoder value", elevatorEncoder.get());
    }

    public void runElevatorDown() {

        leftElevator.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
        rightElevator.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);

        System.out.println(elevatorEncoder.getDistance());
        SmartDashboard.putNumber("encoder value", elevatorEncoder.get());
    }

    public void zero(){
        elevatorEncoder.reset();
    }


    public void goToPosition(ElevatorPosition position) {
        switch (position) {
            case BOTTOM:
                targetPosition = bottomPosition;
                currentState = ElevatorState.MOVING_TO_BOTTOM;
                break;
            case MIDDLE:
                targetPosition = middlePosition;
                currentState = ElevatorState.MOVING_TO_MIDDLE;
                break;
            case TOP:
                targetPosition = topPosition;
                currentState = ElevatorState.MOVING_TO_TOP;
                break;
        }
    }

    public void updateElevator() {
        switch (currentState) {
            case MOVING_TO_BOTTOM:
                if (elevatorEncoder.getDistance() > bottomPosition) {
                    runElevatorDownPID(bottomPosition);
                } else {
                    stopElevator();
                    currentState = ElevatorState.IDLE;
                }
                break;
            case MOVING_TO_MIDDLE:
                if (elevatorEncoder.getDistance() > middlePosition + 0.2) {
                    runElevatorDownPID(middlePosition);
                } else if (elevatorEncoder.getDistance() < middlePosition - 0.2) {
                    runElevatorUpPID(middlePosition);
                } else {
                    stopElevator();
                    currentState = ElevatorState.IDLE;
                }
                break;
            case MOVING_TO_TOP:
                if (elevatorEncoder.getDistance() < topPosition) {
                    runElevatorUpPID(topPosition);
                } else {
                    stopElevator();
                    currentState = ElevatorState.IDLE;
                }
                break;
            case IDLE:
                // Do nothing
                break;
        }
    }

    private void runElevatorUpPID(double setpoint) {
        if (elevatorEncoder.getDistance() >= topPosition) {
            stopElevator();
            return;
        }
        double output = elevatorPID.calculate(elevatorEncoder.getDistance(), setpoint);
        leftElevator.set(output);
        rightElevator.set(output);

        System.out.println(elevatorEncoder.getDistance());
        SmartDashboard.putNumber("encoder value", elevatorEncoder.get());
    }

    private void runElevatorDownPID(double setpoint) {
        if (elevatorEncoder.getDistance() <= bottomPosition) {
            stopElevator();
            return;
        }
        double output = elevatorPID.calculate(elevatorEncoder.getDistance(), setpoint);
        leftElevator.set(output);
        rightElevator.set(output);

        System.out.println(elevatorEncoder.getDistance());
        SmartDashboard.putNumber("encoder value", elevatorEncoder.get());
    }

    public void stopElevator() {
        leftElevator.set(0);
        rightElevator.set(0);
    }
}