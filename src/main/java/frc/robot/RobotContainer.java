// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.LateratorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Home;
import frc.robot.commands.Intake;
import frc.robot.commands.autonomous.RemoveAlgae;
import frc.robot.commands.autonomous.ScoreLV4;

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

    public Trigger intakeTrigger;
    // private final SendableChooser<Command> autoChooser;

    public Trigger elevatorTrigger;


    public RobotContainer() {
        intakeTrigger = new Trigger(Robot.getManipulator() :: hasCoralEntered);
        elevatorTrigger = new Trigger(() -> Robot.getElevator().getBottomLimit());
        // Register Commands for path planner
        registerNamedCommands();
        // manipulatorOUT = new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OUT), Robot.getManipulator());
        // manipulatorOFF = new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator());
        // Configure the button binding
        configureBindings();

    }

    private void registerNamedCommands() {
        //Setting the Commands with names and subsystem commands
        // NamedCommands.registerCommand("removeAlgae", new RemoveAlgae());
        // NamedCommands.registerCommand("scoreLV4", new ScoreLV4());
        // NamedCommands.registerCommand("home", new Home());
    }

    /**
     * Use this method to define your controller->command mappings.
     */
    private void configureBindings() {

        // Robot.getDrivetrain().setDefaultCommand(
        //         new RunCommand(
        //           () -> {
        //             double xSpeed = (Math.abs(driver.getLeftX()) > Constants.Controllers.DEADBAND)
        //                 ? driver.getLeftX()
        //                 : 0;
        //             double ySpeed = (Math.abs(driver.getLeftY()) > Constants.Controllers.DEADBAND)
        //                 ? driver.getLeftY()
        //                 : 0;
        //             double rotSpeed = (Math.abs(driver.getRightX()) > Constants.Controllers.DEADBAND)
        //                 ? driver.getRightX()
        //                 : 0;

        //             Robot.getDrivetrain().drive(ySpeed, xSpeed, rotSpeed, true, true);
        //           }, Robot.getDrivetrain()

        //         ));

        // driver.rightBumper().whileTrue(
        //     new InstantCommand(() -> Robot.getDrivetrain().slowState(true), Robot.getDrivetrain())
        // ).whileFalse(
        //     new InstantCommand(() -> Robot.getDrivetrain().slowState(false), Robot.getDrivetrain())
        // );

        // driver.x().whileTrue(
        //     new InstantCommand(() -> Robot.getDrivetrain().setXState(true), Robot.getDrivetrain())
        // ).whileFalse(
        //     new InstantCommand(() -> Robot.getDrivetrain().setXState(false), Robot.getDrivetrain())
        // );

        // operator.y().onTrue(
        //   new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4), Robot.getDrivetrain())
        // );

        // operator.x().onTrue(
        //   new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV3), Robot.getElevator())
        // );

        // operator.b().onTrue(
        //   new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV2), Robot.getElevator())
        // );

        operator.a().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV1), Robot.getElevator())
        );

        operator.rightBumper().onTrue(
          new ParallelCommandGroup(
            new Home()
          )
        );

        // operator.leftBumper().whileTrue(
        //     // manipulatorOUT.unless(() -> Robot.getManipulator().isCoralReady())
        //     // new ConditionalCommand(manipulatorOUT, manipulatorOFF, () -> !Robot.getManipulator().isCoralReady())
        //     new Intake()
        // ).whileFalse(
        //     new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator())
        // );

        // operator.rightTrigger().whileTrue(
        //   new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.SCORING), Robot.getManipulator())
        // ).whileFalse(
        //     new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator())
        // );

        // Robot.getManipulator().setDefaultCommand(
        //     new RunCommand(() -> { 
        //                 double speed = (Math.abs(operator.getLeftY()) > Constants.Controllers.DEADBAND)
        //                             ? operator.getLeftY()
        //                             : 0;
        //                 Robot.getManipulator().setLateratorRawPower(speed*0.1);
        //             }, Robot.getManipulator()
        // ));

        // Robot.getElevator().setDefaultCommand(
        //     new RunCommand(() -> { 
        //         double speed = (Math.abs(operator.getLeftY()) > Constants.Controllers.DEADBAND)
        //                     ? operator.getLeftY()
        //                     : 0;
        //         Robot.getElevator().setRawPower(-speed*0.3);
        //     }, Robot.getElevator()
        // ));


    }

    // @Override
    // public void periodic() {

    // }

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
      //return autoChooser.getSelected();
      return null;
    }
}
