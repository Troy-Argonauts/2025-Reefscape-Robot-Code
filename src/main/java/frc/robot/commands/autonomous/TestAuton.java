package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

public class TestAuton extends SequentialCommandGroup {
    public TestAuton(){
        super(

            new DriveX(2, Robot.getDrivetrain()).withTimeout(3),

            new WaitCommand(0.5),
            
            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4), Robot.getElevator()),

            new WaitUntilCommand(() -> Robot.getElevator().isPIDFinished()),

            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.FORWARD), Robot.getManipulator()),

            new WaitCommand(1),

            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator()),

            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.HOME), Robot.getElevator())

        );
    }
}