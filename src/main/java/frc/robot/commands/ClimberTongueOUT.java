package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
 
/**
 * Sets tongue state to OUT until the limit switch is hit, when the limit switch is hit it sets tongue state to OFF
 */
public class ClimberTongueOUT extends SequentialCommandGroup {
    public ClimberTongueOUT(){
        super(
                new InstantCommand(() -> Robot.getClimber().setTongueState(Climber.TongueStates.OUT), Robot.getClimber()),

                new WaitUntilCommand(() -> Robot.getClimber().tongueExtended()),

                new InstantCommand(() -> Robot.getClimber().setTongueState(Climber.TongueStates.OFF), Robot.getClimber())
            );
    }
}