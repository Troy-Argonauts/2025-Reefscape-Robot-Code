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

public class Elevator extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;
    private final DigitalInput topLimitSwitch, bottomLimitSwitch;

    private double encoderValue, target, oldtarget = 0;


    private DoubleLogEntry ElevatorLeftOutputCurrentLog;
    private DoubleLogEntry ElevatorRightOutputCurrentLog;

    TalonFXConfiguration config = new TalonFXConfiguration();

    PositionVoltage positionVoltage = new PositionVoltage(0);
    Slot0Configs slot0Configs = new Slot0Configs();

    public Elevator() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

        leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);
        topLimitSwitch = new DigitalInput(Constants.Elevator.TOP_LIMIT_SWITCH_SLOT);
        bottomLimitSwitch = new DigitalInput(Constants.Elevator.BOTTOM_LIMIT_SWITCH_SLOT);

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

    // logs data
    public void logData() {
        SmartDashboard.putNumber("Encoder Value", encoderValue);
        SmartDashboard.putNumber("Left Motor Current", leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Current", rightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("oldTarget", oldtarget);
        
        ElevatorLeftOutputCurrentLog.append(leftMotor.getSupplyCurrent().getValueAsDouble());
        ElevatorRightOutputCurrentLog.append(rightMotor.getSupplyCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        encoderValue = leftMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("target", target);
        logData();
        leftMotor.setControl(positionVoltage.withPosition(target));
        if (getBottomLimitSwitch() == true) {
            resetEncoder();
        }
    }

    // sets raw power for left, right is follower
    public void setRawPower(double power) {
        leftMotor.set(power);
    }

    // returns true if PID is done
    public boolean isPIDFinished() {
        return (Math.abs(target - leftMotor.getPosition().getValueAsDouble()) < 0.01);
    }
    //change the value above

    // returns left motor position as double
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    // returns target
    public double getCurrentTarget() {
        return target;
    }

    // sets the disired target
    public void setDesiredTarget(double target) {
        this.target = target;
    }

    public enum ElevatorStates{
        INTAKE(0),
        LV1(0),
        LV2(0),
        LV3(0),
        LV4(0);
        //change the numbers above
        
        final double elevatorPosition;

        ElevatorStates(double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
        }
    }

    // sets target to the state
    public void setDesiredState(ElevatorStates state) {
        oldtarget= target;
        target = state.elevatorPosition;
    }

    // changes target depending on joystick position
    public void setPower(double joyStickValue) {
        double newTarget = target + (joyStickValue * 20);
        if ((target <= 5 || target >= 0) && newTarget > 0 && target != newTarget) {
            target = newTarget;
        } else if (newTarget < target && bottomLimitSwitch.get()) { // If elevator is moving down (new encoder value is less than current encoder value) and bottomLimitSwitch is not pressed
            target = newTarget;
        } else if (newTarget > target && topLimitSwitch.get()) {
            target = newTarget;
        }
    }

    // returns true if top limit switch is pressed
    public boolean getTopLimitSwitch() {
        return topLimitSwitch.get();
    }

    // returns true if bottom limit switch is pressed
    public boolean getBottomLimitSwitch() {
        return bottomLimitSwitch.get();
    }

    // resets encoders
    public void resetEncoder() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }
}


