import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private TalonFX armMotor1, armMotor2, alignMotor, tongueMotor;
    private DigitalInput armLimit, tongueLimit, alignLimit;
    public Climber() {}
    
}
