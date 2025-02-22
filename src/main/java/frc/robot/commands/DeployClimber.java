package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DeployClimber extends SequentialCommandGroup {
    public DeployClimber(){
        super(
            new ClimberTongueOUT()
        );
    }
}

