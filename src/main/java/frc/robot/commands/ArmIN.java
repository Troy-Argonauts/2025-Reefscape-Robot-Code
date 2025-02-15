package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
 
/**
 * Sets arm state to IN until the limit switch is hit, when the limit switch is hit it sets arm state to OFF
 */
public class ArmIN extends SequentialCommandGroup {
    public ArmIN(){
        super(
                new InstantCommand(() -> Robot.getClimber().setArmState(Climber.ArmStates.IN), Robot.getClimber()),

                new WaitUntilCommand(() -> Robot.getClimber().getArmLimit()),

                new InstantCommand(() -> Robot.getClimber().setArmState(Climber.ArmStates.OFF), Robot.getClimber())
            );
    }
}