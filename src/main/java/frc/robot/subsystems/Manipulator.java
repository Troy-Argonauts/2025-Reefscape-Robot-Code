package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator.*;

public class Manipulator extends SubsystemBase{
    private DoubleLogEntry currentManipulatorVelocityLog;
    private DoubleLogEntry topMotorSupplyCurrentLog;
    private DoubleLogEntry bottomMotorSupplyCurrentLog;

    private double currentManipulatorVelocity;
    private double desiredManipulatorVelocity;

    private double currentLateratorPosition; 
    private double desiredLateratorPosition;

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private TalonFX lateratorMotor;
    
    private DigitalInput limitSwitch;

    private DigitalInput funnel;
    private DigitalInput manipulatorA;
    private DigitalInput manipulatorB;

    private VelocityVoltage velocityVoltage = new VelocityVoltage(null);
    private PositionVoltage positionVoltage = new PositionVoltage(null);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Slot0Configs slot0Config = config.Slot0;

    public Manipulator() {
        topMotor = new TalonFX(Constants.Manipulator.TOP_MOTOR_CAN_ID);
        bottomMotor = new TalonFX(Constants.Manipulator.BOTTOM_MOTOR_CAN_ID);
        lateratorMotor = new TalonFX(Constants.Manipulator.LATERATOR_MOTOR_CAN_ID);

        limitSwitch = new DigitalInput(Constants.Manipulator.LATERATOR_LIMIT_SWITCH);

        funnel = new DigitalInput(Constants.Manipulator.FUNNEL_BEAM_BREAK);
        manipulatorA = new DigitalInput(Constants.Manipulator.MANIPULATOR_A_BEAM_BREAK);
        manipulatorB = new DigitalInput(Constants.Manipulator.MANIPULATOR_B_BEAM_BREAK);

        topMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);
        lateratorMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();
        
        slot0Config.kP = Constants.Manipulator.P;
        slot0Config.kI = Constants.Manipulator.I;
        slot0Config.kD = Constants.Manipulator.D;
        slot0Config.kV = Constants.Manipulator.V;


        bottomMotor.setControl(new Follower(Constants.Manipulator.TOP_MOTOR_CAN_ID, true));

        topConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 30;

        topMotor.getConfigurator().apply(topConfiguration);
        bottomMotor.getConfigurator().apply(bottomConfiguration);

        velocityVoltage.Slot = 0;

        DataLog log = DataLogManager.getLog();

        currentManipulatorVelocityLog = new DoubleLogEntry((log), "Current Manipulator Velocity");
        topMotorSupplyCurrentLog = new DoubleLogEntry((log), "Top Motor Supply Current");
        bottomMotorSupplyCurrentLog = new DoubleLogEntry(log, "Bottom Motor Supply Current");

    }

    @Override
    public void periodic() {

        currentManipulatorVelocityLog.append(currentManipulatorVelocity);
        topMotorSupplyCurrentLog.append(topMotor.getSupplyCurrent().getValueAsDouble());
        bottomMotorSupplyCurrentLog.append(bottomMotor.getSupplyCurrent().getValueAsDouble());    

        SmartDashboard.putNumber("Current Roller Velocity", currentManipulatorVelocity);
        SmartDashboard.putNumber("Desired Roller Velocity", desiredManipulatorVelocity);

        currentManipulatorVelocity = (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble());

        topMotor.setControl(velocityVoltage.withVelocity(desiredManipulatorVelocity));
    }
 
    public enum ManipulatorStates {
        IN(-800),

        OUT(800),

        OFF(0);

        final double manipulatorVelocity;

        ManipulatorStates(double manipulatorVelocity) {
            this.manipulatorVelocity = manipulatorVelocity;
        }

    }

    public enum LateratorStates {
        IN(0),
        OUT(200);

        final double lateratorPosition;
        LateratorStates(double lateratorPosition) {
            this.lateratorPosition = lateratorPosition;
        }
    }

    public void setState(ManipulatorStates state) {
        desiredManipulatorVelocity = state.manipulatorVelocity;
    }

    public boolean isTopPidFinished() {
        return (Math.abs(desiredManipulatorVelocity - getVelocity()) <= 50);
    }

    public boolean isLimitSwitchPressed(){
        return limitSwitch.get();
    }

    public boolean getFunnelBeamBreak() {
        return funnel.get();
    }

    public boolean getManipulatorABeamBreak() {
        return manipulatorA.get();
    }

    public boolean getManipulatorBBeamBreak() {
        return manipulatorB.get();
    }


    // Evan is cool
    public double getVelocity() {
        return (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble()) / 2;
    } 

    public double getManipulatorDesiredVelocity() {
        return desiredManipulatorVelocity;
    }

    public double getLateratorCurrentPosition() {
        return currentLateratorPosition;
    }

    public void setLateratorRawPower(double power) {
        if ((isLimitSwitchPressed() == true && power >= 0) || (getLateratorCurrentPosition() >=  Constants.Manipulator.MAX_LATERATOR_POSITION && power <= 0)) {
            lateratorMotor.set(power);
        }
    }

    public boolean isManipulatorPIDFinished() {
        return (Math.abs(desiredManipulatorVelocity - topMotor.getPosition().getValueAsDouble()) <= 0.03);
    }

    public void setManipulatorDiseredTarget(double target) {
        desiredManipulatorVelocity = target;
    }

    public void setManipulatorState(ManipulatorStates state) {
        desiredManipulatorVelocity = state.manipulatorVelocity;
    }
}