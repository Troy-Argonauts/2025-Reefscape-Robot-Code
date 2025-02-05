package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX motorLeft, motorRight;
    private final DigitalInput limitSwitchTop, limitSwitchBottom;

    private double encoderValue, target = 0;

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
    }

    public void periodic() {
        encoderValue = motorLeft.getPosition().getValueAsDouble();
    }

    public void run() {
        if (target == 0 && encoderValue <= 1){}
        else {} 
    }
    }


