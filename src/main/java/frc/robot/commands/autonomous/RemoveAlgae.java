package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.LateratorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

/**
 * Sets laterator state to OUT, when the laterator is fully extended sets the state to OFF
 */
public class RemoveAlgae extends SequentialCommandGroup {
    public RemoveAlgae(){
        super(  
            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.ALGAE_HIGH), Robot.getElevator()),
                
            new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.OUT), Robot.getManipulator()),

            new WaitUntilCommand(() -> Robot.getElevator().isPIDFinished()),

            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.SCORING), Robot.getManipulator()),

            new WaitUntilCommand(1),

            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator()),

            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.HOME), Robot.getElevator()),

            new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.IN), Robot.getManipulator())

        );
    }
}