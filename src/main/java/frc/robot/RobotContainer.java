// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.LateratorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(Constants.Controllers.DRIVER);
    public static final CommandXboxController operator = new CommandXboxController(Constants.Controllers.OPERATOR);

    public RobotContainer() {
        // Configure the button bindings
        configureBindings();
    }

    /**
     * Use this method to define your controller->command mappings.
     */
    private void configureBindings() {
        Robot.getDrivetrain().setDefaultCommand(
                new RunCommand(
                        () -> {
                            double xSpeed = (Math.abs(driver.getLeftX()) > Constants.Controllers.DEADBAND)
                                    ? driver.getLeftX()
                                    : 0;
                            double ySpeed = (Math.abs(driver.getLeftY()) > Constants.Controllers.DEADBAND)
                                    ? driver.getLeftY()
                                    : 0;
                            double rotSpeed = (Math.abs(driver.getRightX()) > Constants.Controllers.DEADBAND)
                                    ? driver.getRightX()
                                    : 0;

                            Robot.getDrivetrain().drive(ySpeed, xSpeed, rotSpeed, true, true);
                        }, Robot.getDrivetrain()

                ));

        // driver.a().onTrue(
        // new InstantCommand(() -> Robot.getDrivetrain().setToZero())
        // );

        driver.x().whileTrue(
                new InstantCommand(() -> Robot.getDrivetrain().setXState(true))).whileFalse(
                        new InstantCommand(() -> Robot.getDrivetrain().setXState(false)));

        operator.y().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4))
        );

        operator.x().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV3))
        );

        operator.b().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV2))
        );

        operator.a().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV1))
        );

        operator.leftBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV1)),
            new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.IN))
          )
        );

        operator.rightBumper().onTrue(
          new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.IN))
        );

        operator.rightTrigger().onTrue(
          new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OUT))
        );

        operator.povUp().onTrue(
          new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.IN))
        );

        operator.povDown().onTrue(
          new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.OUT))
        );

    }

    public static CommandXboxController getDriver() {
        return driver;
    }

    public static CommandXboxController getOperator() {
        return operator;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
