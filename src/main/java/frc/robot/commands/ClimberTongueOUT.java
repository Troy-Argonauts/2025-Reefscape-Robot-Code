package frc.robot.commands;

import java.util.function.ToIntBiFunction;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Tongue;
import frc.robot.subsystems.Tongue.TongueStates;

/**
 * Sets tongue state to OUT until the limit switch is hit, when the limit switch is hit it sets tongue state to OFF
 */
public class ClimberTongueOUT extends SequentialCommandGroup {
    public ClimberTongueOUT(){
        super(
                // new InstantCommand(() -> Robot.getTongue().setTongueRawPower(0.3), Robot.getClimber()),
                new InstantCommand(() -> Robot.getTongue().setTongueState(Tongue.TongueStates.OUT), Robot.getTongue()),

                new WaitUntilCommand(() -> Robot.getTongue().tongueExtended()),

                new InstantCommand(() -> Robot.getTongue().setTongueRawPower(0), Robot.getTongue())
            );
    }
}