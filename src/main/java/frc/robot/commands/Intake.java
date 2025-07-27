package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

public class Intake extends SequentialCommandGroup {
    public Intake() {
        super(
            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.INTAKE)),
            new WaitUntilCommand(() -> Robot.getManipulator().isCoralReady()),
            new WaitCommand(0.08),
            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF))
        );
    }
}
