// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.LateratorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PathPlanner;
import frc.robot.commands.Home;
import frc.robot.commands.Intake;
import frc.robot.commands.LateratorOUT;
import frc.robot.commands.autonomous.DriveX;
import frc.robot.commands.autonomous.RemoveAlgae;
import frc.robot.commands.autonomous.ScoreLV4;
import frc.robot.commands.autonomous.TestAuton;


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
    public Trigger elevatorTrigger;

    private final SendableChooser<Command> autoChooser;


    public static Command P2_Cross;



    public RobotContainer() {
      // try {
      //   PathPlannerPath P2_Cross_file = PathPlannerPath.fromPathFile("P2_Cross");
      //   P2_Cross = AutoBuilder.followPath(P2_Cross_file);

      // } catch (Exception e) {
      //   DriverStation.reportError("Oops" + e.getMessage(), e.getStackTrace());
      // }

        // intakeTrigger = new Trigger(Robot.getManipulator() :: hasCoralEntered);
        // Register Commands for path planner
        elevatorTrigger = new Trigger(() -> Robot.getElevator().checkHeight());
        registerNamedCommands();
        // manipulatorOUT = new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OUT), Robot.getManipulator());
        // manipulatorOFF = new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator());
        // Configure the button binding
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Do Nothing", new WaitCommand(10.0));
        autoChooser.addOption("TestAuton", new TestAuton());
        autoChooser.addOption("DriveX", new DriveX(2, Robot.getDrivetrain()));
        // autoChooser.addOption("Test P2_Cross", new ScoreLV4());
        // autoChooser.addOption("Test1", P2_Cross);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void registerNamedCommands() {
        //Setting the Commands with names and subsystem commands
        NamedCommands.registerCommand("removeAlgae", new RemoveAlgae());
        NamedCommands.registerCommand("scoreLV4", new ScoreLV4());
        NamedCommands.registerCommand("Place Level 4", new WaitCommand(5));
        // NamedCommands.registerCommand("home", new Home());
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

        // elevatorTrigger.onTrue(
        //   new InstantCommand(() -> Robot.getDrivetrain().slowState(true))
        // ).onFalse(
        //   new InstantCommand(() -> Robot.getDrivetrain().slowState(false))
        // );

        driver.rightBumper().onTrue(
            new InstantCommand(() -> Robot.getDrivetrain().slowState(true), Robot.getDrivetrain())
        ).onFalse(
            new InstantCommand(() -> Robot.getDrivetrain().slowState(false), Robot.getDrivetrain())
        );

        driver.x().whileTrue(
            new InstantCommand(() -> Robot.getDrivetrain().setXState(true), Robot.getDrivetrain())
        ).whileFalse(
            new InstantCommand(() -> Robot.getDrivetrain().setXState(false), Robot.getDrivetrain())
        );

        // driver.a().whileTrue(
        //     new InstantCommand(() -> Robot.getDrivetrain().driveX(2))
        // ).whileFalse(
        //   new InstantCommand(() -> Robot.getDrivetrain().driveX(0))
        // );

        // driver.a().whileTrue(
        // );

        operator.rightBumper().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4), Robot.getDrivetrain())
        );

        operator.y().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV3), Robot.getElevator())
        );

        operator.x().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV2), Robot.getElevator())
        );

        operator.b().onTrue(
          new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV1), Robot.getElevator())
        );

        operator.a().onTrue(
          new ParallelCommandGroup(
            new Home()
          )
        );

        operator.leftTrigger().whileTrue(
            // manipulatorOUT.unless(() -> Robot.getManipulator().isCoralReady())
            // new ConditionalCommand(manipulatorOUT, manipulatorOFF, () -> !Robot.getManipulator().isCoralReady())
            // new Intake()
            new ConditionalCommand(
                new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF)),
                new Intake(),
                () -> Robot.getManipulator().isCoralReady())
        ).whileFalse(
            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator())
        );

        operator.rightTrigger().whileTrue(
          new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.FORWARD), Robot.getManipulator())
        ).whileFalse(
            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator())
        );

        operator.povUp().onTrue(
          new InstantCommand(() -> Robot.getManipulator().setLateratorState(LateratorStates.OFF))
        );

        operator.povDown().onTrue(
          new LateratorOUT()
        );

        operator.povRight().whileTrue(
          new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.REVERSE))
        ).whileFalse(
          new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF))
        );

        // Robot.getManipulator().setDefaultCommand(
        //     new RunCommand(() -> { 
        //                 double speed = (Math.abs(operator.getLeftY()) > Constants.Controllers.DEADBAND)
        //                             ? operator.getLeftY()
        //                             : 0;
        //                 Robot.getManipulator().setLateratorRawPower(speed*0.3);
        //             }, Robot.getManipulator()
        // ));

        Robot.getElevator().setDefaultCommand(
            new RunCommand(() -> {
                Robot.getElevator().adjustSetpoint(-operator.getLeftY());
            }, Robot.getElevator()
        ));

        // Robot.getElevator().setDefaultCommand(
        //     new RunCommand(() -> { 
        //                 double speed = (Math.abs(operator.getLeftY()) > Constants.Controllers.DEADBAND)
        //                             ? operator.getLeftY()
        //                             : 0;
        //                 Robot.getElevator().setRawPower(speed*0.5);
        //             }, Robot.getElevator()
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
      return autoChooser.getSelected();
    }
}
