package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator;

/**
 * Sets laterator state to OUT, when the laterator is fully extended sets the state to OFF
 */
public class LateratorOUT extends SequentialCommandGroup {
    public LateratorOUT(){
        super(
                new InstantCommand(() -> Robot.getManipulator().setLateratorState(Manipulator.LateratorStates.OUT), Robot.getManipulator()),

                new WaitUntilCommand(() -> Robot.getManipulator().lateratorExtended()),

                new InstantCommand(() -> Robot.getManipulator().setLateratorState(Manipulator.LateratorStates.OFF), Robot.getManipulator())
            );
    }
}