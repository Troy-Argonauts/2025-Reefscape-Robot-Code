package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class representing Elevator Subsystem
 * 
 * @author firearcher2012, VedNakum, Evan13019, Kunal Bareth
 */
public class Elevator extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;
    private DigitalInput topLimit, bottomLimit;

    private double encoderValue, target, oldtarget = 0;


    private DoubleLogEntry ElevatorLeftOutputCurrentLog;
    private DoubleLogEntry ElevatorRightOutputCurrentLog;

    TalonFXConfiguration config = new TalonFXConfiguration();

    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    Slot0Configs slot0Configs = new Slot0Configs();

    /**
     * Instantiates motors and limit switches; Set neutral modes; Assigns PID constants.
     */
    public Elevator() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

        leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);
        topLimit = new DigitalInput(Constants.Elevator.TOP_LIMIT_SWITCH_SLOT);
        bottomLimit = new DigitalInput(Constants.Elevator.BOTTOM_LIMIT_SWITCH_SLOT);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;

        slot0Configs.kP = Constants.Elevator.P;
        slot0Configs.kI = Constants.Elevator.I;
        slot0Configs.kD = Constants.Elevator.D;
        slot0Configs.kV = Constants.Elevator.V;
        slot0Configs.kG = Constants.Elevator.G;

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        rightMotor.setControl(new Follower(Constants.Elevator.LEFT_MOTOR_ID, true));

        DataLog log = DataLogManager.getLog();

        ElevatorLeftOutputCurrentLog = new DoubleLogEntry(log, "ElevatorLeftOutputCurrent");
        ElevatorRightOutputCurrentLog = new DoubleLogEntry(log, "ElevatorRightOutputCurrent");
    }

   
    /**
     * Gets encoder value; Logs data; Sets position setpoint to target; Resets encoder.
     */
    @Override
    public void periodic() {
        encoderValue = leftMotor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("target", target);
        SmartDashboard.putNumber("Encoder Value", encoderValue);
        SmartDashboard.putNumber("Left Motor Current", leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Current", rightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("oldTarget", oldtarget);
        
        ElevatorLeftOutputCurrentLog.append(leftMotor.getSupplyCurrent().getValueAsDouble());
        ElevatorRightOutputCurrentLog.append(rightMotor.getSupplyCurrent().getValueAsDouble());

        leftMotor.setControl(positionVoltage.withPosition(target));

        if (getBottomLimit() == true) {
            resetEncoder();
        }
    }

    /**
     * Sets raw power for left, right is follower 
     * @param power desired power from -1 to 1
     */
    public void setRawPower(double power) {
        leftMotor.set(power);
    }

    /**
     * Checks if PID value for left motor is within given range
     * @return whether PID is finished
     */
    public boolean isPIDFinished() {
        return (Math.abs(target - leftMotor.getPosition().getValueAsDouble()) < 0.01);
    }
    //change the value above

    /**
     * Gets motor position
     * @return left motor position as double
     */
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns value of current elevator target position
     * @return current elevator target
     */
    public double getCurrentTarget() {
        return target;
    }

    /**
     * Sets the desired elevator target
     * @param target desired elevator target
     */
    public void setDesiredTarget(double target) {
        this.target = target;
    }

    /**
     * Sets enumerators for encoder positions of various Elevator States
     */
    public enum ElevatorStates{
        HOME(0),
        LV1(0),
        LV2(0),
        LV3(0),
        LV4(0),
        ALGAE_HIGH(0),
        ALGAE_LOW(0);
        //change the numbers above
        
        final double elevatorPosition;

        ElevatorStates(double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
        }
    }

    /**
     * Sets target as the desired Elevator state
     * @param state elevator state
     */
    public void setDesiredState(ElevatorStates state) {
        oldtarget= target;
        target = state.elevatorPosition;
    }

    /**
     * Changes target depending on joystick position
     * @param joyStickValue joystick value between 1 and -1
     */
    public void adjustSetpoint(double joyStickValue) {
        double newTarget = target + (joyStickValue * 20);
        if ((target <= 5 || target >= 0) && newTarget > 0 && target != newTarget) {
            target = newTarget;
        } else if (newTarget > target && getBottomLimit()) { // If elevator is moving down (new encoder value is less than current encoder value) and bottomLimitSwitch is not pressed
            target = newTarget;
        } else if (newTarget < target && getTopLimit()) {
            target = newTarget;
        }
    }

    /**
     * Returns whether the top limit switch is pressed
     * @return whether it is pressed
     */
    public boolean getTopLimit() {
        return topLimit.get();
    }

    /**
     * Returns whether the bottom limit switch is pressed
     */
    public boolean getBottomLimit() {
        return bottomLimit.get();
    }

    /**
     * Resets encoders
     */
    public void resetEncoder() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }
}


