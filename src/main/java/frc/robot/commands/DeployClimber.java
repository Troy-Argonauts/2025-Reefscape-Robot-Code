package frc.robot.commands;

import java.security.AllPermission;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class DeployClimber extends SequentialCommandGroup {
    public DeployClimber(){
        super(
            new ClimberArmIN(),
            new ClimberArmOUT(), 
            new WaitUntilCommand(() -> Robot.getClimber().armExtended()), 
            new ClimberTongueOUT()
        );
    }
}

