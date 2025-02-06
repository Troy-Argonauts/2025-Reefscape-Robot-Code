package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX motorLeft, motorRight;
    private final DigitalInput limitSwitchTop, limitSwitchBottom;

    private double encoderValue, target = 0;
    private double oldtarget;

    private DoubleLogEntry ElevatorLeftOutputCurrentLog;
    private DoubleLogEntry ElevatorRightOutputCurrentLog;

    public Elevator() {
        TalonFXConfiguration motorLeftConfigs = new TalonFXConfiguration();
        TalonFXConfiguration motorRightConfigs = new TalonFXConfiguration();

        motorLeft = new TalonFX(1);
        motorRight = new TalonFX(2);
        limitSwitchTop = new DigitalInput(0);
        limitSwitchBottom = new DigitalInput(1);

        motorLeft.setNeutralMode(NeutralModeValue.Brake);
        motorRight.setNeutralMode(NeutralModeValue.Brake);

        motorLeftConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorRightConfigs.CurrentLimits.SupplyCurrentLimit = 40;

        motorLeft.getConfigurator().apply(motorLeftConfigs);
        motorRight.getConfigurator().apply(motorRightConfigs);

        motorRight.setControl(new Follower(1, true));

        ElevatorLeftOutputCurrentLog = new DoubleLogEntry(null, "ElevatorLeftOutputCurrent");
        ElevatorRightOutputCurrentLog = new DoubleLogEntry(null, "ElevatorRightOutputCurrent");

    }

    public void logData() {
        encoderValue = motorLeft.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Encoder Value", encoderValue);
        SmartDashboard.putNumber("Left Motor Current", motorLeft.getSupplyCurrent());
        SmartDashboard.putNumber("Right Motor Current", motorRight.getSupplyCurrent());
        SmartDashboard.putNumber("oldTarget", oldtarget);
        
        ElevatorLeftOutputCurrentLog.append(motorLeft.getStatorCurrent().getValue());
        ElevatorRightOutputCurrentLog.append(motorRight.getStatorCurrent().getValue());
    }

    @Override
    public void periodic() {
        encoderValue = motorLeft.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Encoder Value", encoderValue);
        SmartDashboard.putNumber("target", target);
        logData();
    }

    public void setPower(double setPower) {
        motorLeft.set(setPower);
        motorRight.set(setPower);
    }

    public boolean isPIDFinished() {
        return (Math.abs(target - motorLeft.getPosition().getValueAsDouble()) < 0.01);
    }
    //change the value above

    public double getEncoderValue() {
        return motorLeft.getPosition().getValueAsDouble();
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


    public void run() {
        if (target == 0 && encoderValue <= 1){}
        else {} 
    }
    }


