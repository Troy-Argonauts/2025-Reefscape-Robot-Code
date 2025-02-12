package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator;
 
/**
 * Sets laterator state to IN until the limit switch is hit, when the limit switch is hit it sets laterator state to OFF
 */
public class LateratorIN extends SequentialCommandGroup {
    public LateratorIN(){
        super(
                new InstantCommand(() -> Robot.getManipulator().setLateratorState(Manipulator.LateratorStates.IN), Robot.getManipulator()),

                new WaitUntilCommand(() -> Robot.getManipulator().getLateratorLimit()),

                new InstantCommand(() -> Robot.getManipulator().setLateratorState(Manipulator.LateratorStates.OFF), Robot.getManipulator())
            );
    }
}