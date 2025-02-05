import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.AbsoluteEncoder; 

public class ElevatorSubsystem extends SubsystemBase {
//create elevator motors 
private CanSparkMax m_elevatorMotor; 
private CanSparkMax m_elevatorFollower; 

private AbsoluteElevator m_elevatorEncoder; 
 
public Elevator() {

    //value for bottom of elevator 
    this.m_elevatorMotor = new CanSparkMax (port, type); 
    this.m_elevatorFollower = new CanSparkMax(port, MotorType); 

    this.m_elevatorFollower.follow(this.m_elevatorMotor); 

    this.m_elevatorFollower.setInverted(true); 
    this.m_elevatorEncoder = new DutyCycleEncoder (0,4.0,2.0); 
}

public void ElevatorUp(){
    this.m_elevatorMotor.set(0.2);
}

public void ElevatorDown(){
    this.m_elevatorMotor.set(-0.2); 
}

public double ElevatorTestValue(){
    m_elevetorEncoder.get(); 
    System.out.println (m_elevatorEncoder.get()); 
}

}
