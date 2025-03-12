package frc.robot.subsystems;

// Necessary imports for our SwerveModule
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.SwerveModule.*;

/** 
 * Class representing an individual Swerve Module.
*/
public class SwerveModule extends SubsystemBase{
    private TalonFX driveMotor, turnMotor;
    public double driveValue;
    private CANcoder turnEncoder;
    public double turnEncoderValue = 0;
    public double turnEncoderRotation = 0;

    private TalonFXConfiguration config = new TalonFXConfiguration();

    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(1);
    VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private Slot0Configs driveConfig = config.Slot0;
    private Slot1Configs turnConfig = config.Slot1;

    public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double chassisAngularOffset;

    /** 
     * <ul>
     *  <li>Instantiates motors and sensors; </li>
     *  <li>Assigns CAN IDs. </li>
     *  <li>Sets current limits, Neutral Modes, and Inversion direction for motors. </li>
     *  <li>Configures CANcoder. </li>
     *  <li>Configures PID control mode and associated constants. </li>
     * </ul>
     *
     * @param driveMotorID        ID for the drive motor
     * @param turnMotorID         ID for the turn motor
     * @param turnEncoderID       ID for the turn encoder
     * @param canbusName          CAN bus name
     * @param chassisAngularOffset Angular offset for chassis
     * @param driveInverted       Whether the drive motor is inverted
    */
    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, String canbusName, double chassisAngularOffset, boolean driveInverted){
        driveMotor = new TalonFX(driveMotorID, canbusName);
        turnMotor = new TalonFX(turnMotorID, canbusName);
        driveValue = 0;
        turnEncoder = new CANcoder(turnEncoderID, canbusName);

        CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
        encoderConfigs.MagnetSensor.MagnetOffset = chassisAngularOffset; // make sure this is in rotations
        turnEncoder.getConfigurator().apply(encoderConfigs);

        TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
        turnConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfigs.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnConfigs.CurrentLimits.SupplyCurrentLimit = TURNING_MOTOR_CURRENT_LIMIT;
        turnConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotor.getConfigurator().apply(turnConfigs);

        driveConfig.kP = DRIVE_P;
        driveConfig.kI = DRIVE_I;
        driveConfig.kD = DRIVE_D;
        driveConfig.kS = DRIVE_S;
        driveConfig.kV = DRIVE_V;

        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

        if (driveInverted){
            driveConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            driveConfigs.CurrentLimits.SupplyCurrentLimit = DRIVING_MOTOR_CURRENT_LIMIT;
            driveMotor.getConfigurator().apply(driveConfigs);
        } else {
            driveConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            driveConfigs.CurrentLimits.SupplyCurrentLimit = DRIVING_MOTOR_CURRENT_LIMIT;
            driveMotor.getConfigurator().apply(driveConfigs);
        }
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        turnConfig.kP = TURN_P;
        turnConfig.kI = TURN_I;
        turnConfig.kD = TURN_D;
        turnConfig.kS = TURN_S;

        turnMotor.getConfigurator().apply(turnConfig);
        driveMotor.getConfigurator().apply(driveConfig);

        //RESET ENCODERS
        resetDriveEncoder();

        this.chassisAngularOffset = chassisAngularOffset;
    }

    /**
     * <ul>
     *  <li>This method is called periodically by the Command Scheduler. </li>
     *  <li>Sets global drive encoder value and turn encoder variables. </li>
     *  <li>Outputs values to SmartDashboard. </li>
     * </ul>
     */
    @Override
    public void periodic() {
        driveValue = driveMotor.getPosition().getValueAsDouble();
        turnEncoderValue = getAngle();
        turnEncoderRotation = turnEncoder.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Desired State Rotation", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Desired State Speed", desiredState.speedMetersPerSecond);
    }

   /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(getVelocity(),
            new Rotation2d(turnEncoderValue - chassisAngularOffset));
    }

   /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            driveValue,
            new Rotation2d(turnEncoderValue - chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        SmartDashboard.putNumber("Desired State Angle2",desiredState.angle.getDegrees());
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d.fromDegrees(turnEncoderValue));

        // Command driving and turning motors towards their respective setpoints (velocity and position).
        driveMotor.setControl(velocityVoltage.withVelocity(correctedDesiredState.speedMetersPerSecond / WHEEL_CIRCUMFERENCE_METERS));
        turnMotor.setControl(positionVoltage.withPosition(correctedDesiredState.angle.getDegrees() / 360));

        SmartDashboard.putNumber("Target Position", correctedDesiredState.angle.getDegrees() / 360);

        this.desiredState = desiredState;
    }

    // /**
    // * Configures feedback for the turn encoder.
    // */
    // public void configureFeedback() {
    //     CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
    //     encoderConfigs.MagnetSensor.MagnetOffset = chassisAngularOffset; // make sure this is in rotations

    //     TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
    //     turnConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //     turnConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    //     turnConfigs.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
    //     turnConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // }


    /** Zeroes the driveMotor encoder on the SwerveModule */
    public void resetDriveEncoder(){
        driveMotor.setPosition(0);
    }

    /** Zeroes the turnMotor encoder on the SwerveModule */
    public void resetTurnEncoder(){
        turnEncoder.setPosition(0);
    }

    /**
     * Retrieves the velocity of the drive motor in meters per second.
     * @return velocity of the drive motor in meters per second
     */
    public double getVelocity(){
        return driveMotor.getVelocity().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS * WHEEL_CIRCUMFERENCE_METERS;
    }

    /** Retrieves the angle of the turn encoder in degrees.
     * @return turn encoder value converted to degrees
    */
    public double getAngle(){
        return turnEncoder.getPosition().getValueAsDouble() * 360;
    }
}