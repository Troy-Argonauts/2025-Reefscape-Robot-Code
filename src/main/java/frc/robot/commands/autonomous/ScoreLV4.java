package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

/**
 * Sets laterator state to OUT, when the laterator is fully extended sets the state to OFF
 */
public class ScoreLV4 extends SequentialCommandGroup {
    public ScoreLV4(){
        super(  
                // RobotContainer.P2_Cross,

                // new WaitCommand(1.7),

                new InstantCommand(() -> Robot.getDrivetrain().drive(0, 0, 0, false, true)),
                // new InstantCommand(() ->   System.out.println("PRINTLEVEL4")),
                new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4), Robot.getElevator()),

                new WaitUntilCommand(() -> Robot.getElevator().isPIDFinished()),

                new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.FORWARD), Robot.getManipulator()),

                new WaitCommand(1),

                new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator()),

                new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.HOME), Robot.getElevator())


            );
    }
}
