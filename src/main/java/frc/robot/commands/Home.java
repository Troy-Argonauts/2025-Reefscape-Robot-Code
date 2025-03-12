package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.LateratorStates;

/**
 * Sets elevator state to HOME position. 
 */
public class Home extends SequentialCommandGroup{
    public Home(){
        super(
            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.HOME), Robot.getElevator())
            // new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.IN), Robot.getManipulator())
        );
    }
}
