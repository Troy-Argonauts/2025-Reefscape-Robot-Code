package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;
    private final DigitalInput topLimitSwitch, bottomLimitSwitch;

    private double encoderValue, target = 0;
    private double oldtarget;

    private DoubleLogEntry ElevatorLeftOutputCurrentLog;
    private DoubleLogEntry ElevatorRightOutputCurrentLog;

    TalonFXConfiguration config = new TalonFXConfiguration();

    PositionVoltage positionVoltage = new PositionVoltage(null);
    Slot0Configs slot0Configs = new Slot0Configs();

    public Elevator() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

        leftMotor = new TalonFX(1);
        rightMotor = new TalonFX(2);
        topLimitSwitch = new DigitalInput(0);
        bottomLimitSwitch = new DigitalInput(1);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;

        slot0Configs.kP = P;
        slot0Configs.kI = I;
        slot0Configs.kD = D;
        slot0Configs.kV = V;
        slot0Configs.kG = G;

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        positionVoltage.Slot = 0;

        rightMotor.setControl(new Follower(1, true));

        ElevatorLeftOutputCurrentLog = new DoubleLogEntry(null, "ElevatorLeftOutputCurrent");
        ElevatorRightOutputCurrentLog = new DoubleLogEntry(null, "ElevatorRightOutputCurrent");
    


    }

    public void logData() {
        encoderValue = leftMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Encoder Value", encoderValue);
        SmartDashboard.putNumber("Left Motor Current", leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Current", rightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("oldTarget", oldtarget);
        
        ElevatorLeftOutputCurrentLog.append(leftMotor.getStatorCurrent().getValueAsDouble());
        ElevatorRightOutputCurrentLog.append(rightMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        encoderValue = leftMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("target", target);
        logData();
        leftMotor.setControl(positionVoltage.withPosition(target));
    }

    public void setRawPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public boolean isPIDFinished() {
        return (Math.abs(target - leftMotor.getPosition().getValueAsDouble()) < 0.01);
    }
    //change the value above

    public double getEncoderValue() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public double getCurrentTarget() {
        return target;
    }

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

    public void setState(ElevatorStates state) {
         oldtarget= target;
        target = state.elevatorPosition;
    }

    public void adjustSetpoint(double joystickValue) {
        if ((joystickValue < 0) && (!topLimitSwitch.get())) {
            target += (joystickValue * 0.2);
        }

        if ((joystickValue > 0) && (!bottomLimitSwitch.get())) {
            target += (joystickValue * 0.2);
        }
    }


    public void run() {
        if (target == 0 && encoderValue <= 1){}
        else {} 
    }

    public boolean getTopLimitSwitch() {
        return topLimitSwitch.get();
    }

    public boolean getBottomLimitSwitch() {
        return bottomLimitSwitch.get();
    }
    }


