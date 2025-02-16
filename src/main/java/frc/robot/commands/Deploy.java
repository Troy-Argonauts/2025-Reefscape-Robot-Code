import java.security.AllPermission;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class Deploy extends SequentialCommandGroup {
    public Deploy(){
        super(
            new ArmIN(),
            new ArmOUT()
        );
    }
}

