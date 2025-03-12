package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class representing the Elevator Subsystem
 * 
 * @author firearcher2012, VedNakum, Evan13019, Kunal Bareth
 */
public class Elevator extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;
    private DigitalInput bottomLimit;

    private double encoderValue, target;
    private double oldTarget = 0;

    private DoubleLogEntry elevatorLeftOutputCurrentLog;
    private DoubleLogEntry elevatorRightOutputCurrentLog;
    private DoubleLogEntry elevatorEncoderLog;

    boolean limitTriggered = false;
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    SoftwareLimitSwitchConfigs limitConfig = new SoftwareLimitSwitchConfigs();

    MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0).withSlot(0);
    Slot0Configs slot0Configs = new Slot0Configs();

    /**
     * <ul>
     *   <li>Instantiates motors and limit switch.</li>
     *   <li>Assigns CAN IDs and Sensor Slots.</li>
     *   <li>Sets current Limits, Nuetral Modes, and Inversion direction for motors.</li>
     *   <li>Sets Right Motor to follow Left; Cofigures PID control mode and associated constants.</li>
     *   <li>Creates data log objects.</li>
     * </ul>
     */
    public Elevator() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

        leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);
        bottomLimit = new DigitalInput(Constants.Elevator.BOTTOM_LIMIT_SWITCH_SLOT);

        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // leftMotorConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
        // rightMotorConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;

        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        slot0Configs.kP = Constants.Elevator.P;
        slot0Configs.kI = Constants.Elevator.I;
        slot0Configs.kD = Constants.Elevator.D;
        slot0Configs.kV = Constants.Elevator.V;
        slot0Configs.kG = Constants.Elevator.G;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration = Constants.Elevator.MOTION_MAGIC_ACCEL;
        motionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;


        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        rightMotor.setControl(new Follower(Constants.Elevator.LEFT_MOTOR_ID, true));
        leftMotor.getConfigurator().apply(slot0Configs);
        leftMotor.getConfigurator().apply(motionMagic);


        DataLog log = DataLogManager.getLog();

        elevatorLeftOutputCurrentLog = new DoubleLogEntry(log, "Elevator Left Output Current");
        elevatorRightOutputCurrentLog = new DoubleLogEntry(log, "Elevator Right Output Current");
        elevatorEncoderLog = new DoubleLogEntry(log, "Elevator Encoder Value");
    }

   
    /**
     * This method is called periodically by the Command Scheduler. 
     * <ul>
     *  <li>Sets global encoderValue variable. </li>
     *  <li>Outputs values to SmartDashboard. </li>
     *  <li>Sets PID control target value and sends PID control request. </li>
     *  <li>Resets encoders using toggle based on bottom limit switch. </li>
     *  <li>Appends logs with corresponding data. </li>
     * </ul>
     */
    @Override
    public void periodic() {
        encoderValue = leftMotor.getPosition().getValueAsDouble();

        // SmartDashboard.putNumber("target", target);
        SmartDashboard.putNumber("Elevator Encoder Value", encoderValue);
        // SmartDashboard.putNumber("Elevator Left Motor Current", leftMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Elevator Right Motor Current", rightMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Elevator oldTarget", oldTarget);
        SmartDashboard.putBoolean("Elevator Limit", getBottomLimit());
        SmartDashboard.putBoolean("Limit Triggered", limitTriggered);
        SmartDashboard.putNumber("Elevator Voltage", leftMotor.getMotorVoltage().getValueAsDouble());

        
        elevatorLeftOutputCurrentLog.append(leftMotor.getSupplyCurrent().getValueAsDouble());
        elevatorRightOutputCurrentLog.append(rightMotor.getSupplyCurrent().getValueAsDouble());
        elevatorEncoderLog.append(leftMotor.getPosition().getValueAsDouble());

        // leftMotor.setControl(mmVoltage.withPosition(target));

        SmartDashboard.putBoolean("Limit Switch Value", getBottomLimit() == true);
        if (getBottomLimit() == true) {
            if (!limitTriggered) {
                resetEncoders();
                System.out.println("Reset Elevator Encoders");
                limitTriggered = true;
            } 
        } else {
            limitTriggered = false;
        }
    }

    /**
     * Sets the elevator motors to a percentage of available voltage.
     * @param power The desired percentage value between -1 and 1
     */
    public void setRawPower(double power) {
        leftMotor.set(power);
    }

    /**
     * Checks if current Elevator position (from left motor) is within
     * 0.2 motor ortations of the current target.
     * @return true if current position is within threshold, false if not
     */
    public boolean isPIDFinished() {
        return (Math.abs(target - leftMotor.getPosition().getValueAsDouble()) < 0.01);
    }


    /**
     * Retrieves the current Elevator position.
     * @return The current position of the left motor in rotations
     */
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Retrieves the current Elevator target position.
     * @return The current target position in motor rotations
     */
    public double getCurrentTarget() {
        return target;
    }

    /**
     * Sets the desired Elevator target position.
     * @param target The desired Elevator target in motor rotations
     */
    public void setDesiredTarget(double target) {
        this.target = target;
    }

    
    /**
     * Represents the different states of the Elevator subsystem with predefined positions. 
     * Each state corresponds to a specific position value
     */
    public enum ElevatorStates{
        /**
         * Elevator position to intake a Coral.
         */
        HOME(0),
        /**
         * Elevator position to score a Coral on Level 1 of the Reef.
         */
        LV1(30),
        /**
         * Elevator position to score a Coral on Level 2 of the Reef.
         */
        LV2(0),
        /**
         * Elevator position to score a Coral on Level 3 of the Reef.
         */
        LV3(0),
        /**
         * Elevator position to remove an Algae in the lowest position on the Reef.
         */
        ALGAE_LOW(0), 
        /**
         * Elevator position to remove an Algae in the highest position on the Reef.
         */ 
        ALGAE_HIGH(0), 
        /**
         * Elevator position to score a Coral on Level 2 of the Reef.
         */                                                                          
        LV4(0);
        
        final double elevatorPosition;

        ElevatorStates(double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
        }
    }

    /**
     * Updates the Elevator’s target position based on the desired state.
     * @param state he desired {@code ElevatorStates} to set as the target for the Elevator PID control.
     */
    public void setDesiredState(ElevatorStates state) {
        oldTarget = target;
        target = state.elevatorPosition;
    }

    /**
     * Changes Elevator target incrementally based on value from a controller joystick. Includes limiting based on minimum and maximum position values.
     * @param joyStickValue The joystick value between -1 and 1
     */
    public void adjustSetpoint(double joyStickValue) {
        double deadbanded = (Math.abs(joyStickValue) > Constants.Controllers.DEADBAND)
                            ? joyStickValue
                            : 0;
        double newTarget = target + (deadbanded * 0.2);
        if ((newTarget >= target) || (newTarget <= target && !getBottomLimit())){
            oldTarget = target;
            target = newTarget;
        }
    }

    /**
     * Returns whether the bottom limit switch is pressed.
     * @return true if limit is detected, false if not
     */
    public boolean getBottomLimit() {
        return !bottomLimit.get();
    }

    /**
     * Reset motor encoder values to 0.
     */
    public void resetEncoders() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }
}


