package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
 
/**
 * Sets tongue state to IN until the limit switch is hit, when the limit switch is hit it sets tongue state to OFF
 */
public class TongueIN extends SequentialCommandGroup {
    public TongueIN(){
        super(
                new InstantCommand(() -> Robot.getClimber().setTongueState(Climber.TongueStates.IN), Robot.getClimber()),

                new WaitUntilCommand(() -> Robot.getClimber().getArmLimit()),

                new InstantCommand(() -> Robot.getClimber().setTongueState(Climber.TongueStates.OFF), Robot.getClimber())
            );
    }
}