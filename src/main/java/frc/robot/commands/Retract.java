import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class Retract extends SequentialCommandGroup {
    public Retract(){
        super(
                new TongueIN(),
                new ArmIN(),
                new TongueOUT()
            );
    }
}

/*
* if limitswitch then reset encoder
 * wait unitl arm extended
 * 
 */