package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
 
/**
 * Sets tongue state to OUT until the limit switch is hit, when the limit switch is hit it sets tongue state to OFF
 */
public class TongueOUT extends SequentialCommandGroup {
    public TongueOUT(){
        super(
                new InstantCommand(() -> Robot.getClimber().setTongueState(Climber.TongueStates.OUT), Robot.getClimber()),

                new WaitUntilCommand(() -> Robot.getClimber().getArmLimit()),

                new InstantCommand(() -> Robot.getClimber().setTongueState(Climber.TongueStates.OFF), Robot.getClimber())
            );
    }
}