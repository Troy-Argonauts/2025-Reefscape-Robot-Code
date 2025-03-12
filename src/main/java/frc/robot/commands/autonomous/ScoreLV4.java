package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

/**
 * Sets elevator to level 4 reef position, sets manipulator state to score, wait until coral is scored, turn manipulator off and set elevator to home position.
 */
public class ScoreLV4 extends SequentialCommandGroup {
    public ScoreLV4(){
        super(  
                new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4), Robot.getElevator()),

                new WaitUntilCommand(() -> Robot.getElevator().isPIDFinished()),

                new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.SCORING), Robot.getManipulator()),

                new WaitCommand(1),

                new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator()),

                new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.HOME), Robot.getElevator())

            );
    }
}