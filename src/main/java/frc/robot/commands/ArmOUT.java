package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
 
/**
 * Sets arm state to OUT until the limit switch is hit, when the limit switch is hit it sets arm state to OFF
 */
public class ArmOUT extends SequentialCommandGroup {
    public ArmOUT(){
        super(
                new InstantCommand(() -> Robot.getClimber().setArmState(Climber.ArmStates.OUT), Robot.getClimber()),

                new WaitUntilCommand(() -> Robot.getClimber().getArmLimit()),

                new InstantCommand(() -> Robot.getClimber().setArmState(Climber.ArmStates.OFF), Robot.getClimber())
            );
    }
}