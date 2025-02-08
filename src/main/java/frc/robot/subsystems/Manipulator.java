package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
 * Handles the manipulator, including motors, sensors, and states.
 */
public class Manipulator extends SubsystemBase {
    private DoubleLogEntry currentManipRPMLog;
    private DoubleLogEntry topMotorSupplyCurrentLog;
    private DoubleLogEntry bottomMotorSupplyCurrentLog;
    private DoubleLogEntry lateratorMotorSupplyCurrentLog;

    private double currentManipRPM;
    private double manipTargetRPM;

    private double lateratorCurrentPosition; 
    private double lateratorTarget;

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private TalonFX lateratorMotor;
    
    private DigitalInput limitSwitch;

    private DigitalInput funnel;
    private DigitalInput manipulatorA;
    private DigitalInput manipulatorB;

    private ManipulatorStates manipulatorState = ManipulatorStates.OFF;

    private final CoastOut coastRequest = new CoastOut();

    private VelocityVoltage velocityVoltage = new VelocityVoltage(null);
    private PositionVoltage positionVoltage = new PositionVoltage(null);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Slot0Configs slot0Config = config.Slot0;

    /**
     * Constructor for the Manipulator class. Initializes motors, sensors, and configurations.
     */
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
        TalonFXConfiguration lateratorConfiguration = new TalonFXConfiguration();
        
        slot0Config.kP = Constants.Manipulator.P;
        slot0Config.kI = Constants.Manipulator.I;
        slot0Config.kD = Constants.Manipulator.D;
        slot0Config.kV = Constants.Manipulator.V;


        bottomMotor.setControl(new Follower(Constants.Manipulator.TOP_MOTOR_CAN_ID, false));

        topConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        lateratorConfiguration.CurrentLimits.SupplyCurrentLimit = 20;

        topMotor.getConfigurator().apply(topConfiguration);
        bottomMotor.getConfigurator().apply(bottomConfiguration);
        lateratorMotor.getConfigurator().apply(lateratorConfiguration);

        velocityVoltage.Slot = 0;

        DataLog RPM = DataLogManager.getLog();

        currentManipRPMLog = new DoubleLogEntry((RPM), "Current Manipulator Velocity");
        topMotorSupplyCurrentLog = new DoubleLogEntry((RPM), "Top Motor Supply Current");
        bottomMotorSupplyCurrentLog = new DoubleLogEntry(RPM, "Bottom Motor Supply Current");
        lateratorMotorSupplyCurrentLog = new DoubleLogEntry(RPM, "Laterator Motor Supply Current");

    }

     /**
     * This method is called periodically by the scheduler. Logs current values and updates 
     * SmartDashboard with current and target RPMs.
     */
    @Override
    public void periodic() {

        currentManipRPMLog.append(currentManipRPM);
        topMotorSupplyCurrentLog.append(topMotor.getSupplyCurrent().getValueAsDouble());
        bottomMotorSupplyCurrentLog.append(bottomMotor.getSupplyCurrent().getValueAsDouble());    

        SmartDashboard.putNumber("Current Roller Velocity", currentManipRPM);
        SmartDashboard.putNumber("Desired Roller Velocity", manipTargetRPM);

        currentManipRPM = (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble());

        topMotor.setControl(velocityVoltage.withVelocity(manipTargetRPM));
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

    /**
     * Sets the manipulator's state.
     * @param state The desired state of the manipulator.
     */
    public void setState(ManipulatorStates state) {
        manipTargetRPM = state.manipulatorVelocity;
    }

    /**
     * Checks if the top PID loop has finished.
     * @return true if the top PID loop has finished, false otherwise.
     */
    public boolean isTopPidFinished() {
        return (Math.abs(manipTargetRPM - getVelocity()) <= 50);
    }

    /**
     * Checks if the limit switch is pressed.
     * @return true if the limit switch is pressed, false otherwise.
     */
    public boolean isLimitSwitchPressed(){
        return limitSwitch.get();
    }

    /**
     * Checks if the funnel beam break is detected.
     * @return true if the funnel beam break is detected, false otherwise.
     */
    public boolean getFunnelBeamBreak() {
        return funnel.get();
    }

    /**
     * Checks if the manipulator A beam break is detected.
     * @return true if the manipulator A beam break is detected, false otherwise.
     */
    public boolean getManipulatorABeamBreak() {
        return manipulatorA.get();
    }

    /**
     * Checks if the manipulator B beam break is detected.
     * @return true if the manipulator B beam break is detected, false otherwise.
     */
    public boolean getManipulatorBBeamBreak() {
        return manipulatorB.get();
    }


    // Evan is cool***!!!
    /**
     * Gets the current velocity of the manipulator.
     * @return The current velocity of the manipulator in RPM.
     */
    public double getVelocity() {
        return (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble()) / (2 * 60);
    } 

    /**
     * Gets the desired velocity of the manipulator.
     * @return The desired velocity of the manipulator in RPM.
     */
    public double getManipulatorDesiredVelocity() {
        return manipTargetRPM;
    }

    /**
     * Gets the current position of the laterator.
     * @return The current position of the laterator.
     */
    public double getLateratorCurrentPosition() {
        return lateratorCurrentPosition;
    }

    /**
     * Sets the raw power for the laterator motor.
     * @param power The power to set for the laterator motor.
     */
    public void setLateratorRawPower(double power) {
        if ((isLimitSwitchPressed() == true && power >= 0) || (getLateratorCurrentPosition() >=  Constants.Manipulator.MAX_LATERATOR_POSITION && power <= 0)) {
            lateratorMotor.set(power);
        }
    }

    /**
     * gets is manipulator motor PID is finished
     * @return if manipulator motor PID is finished
     */
    public boolean isManipulatorPIDFinished() {
        return (Math.abs(manipTargetRPM - topMotor.getPosition().getValueAsDouble()) <= 0.03);
    }

    /**
     * sets the manipulator motors disered target
     * @param target the target to set for the manipulator motor
     */
    public void setManipulatorDiseredTarget(double target) {
        manipTargetRPM = target;
    }

    /**
     * sets the manipulator motors state
     * @param state the state to set for the manipulator motor
     */
    public void setManipulatorState(ManipulatorStates state) {
        manipTargetRPM = state.manipulatorVelocity;
        manipulatorState = state;
    }

    /**
     * applies rpm and coasts when state off
     */
    public void run() {
        if (manipulatorState == ManipulatorStates.OFF) {
            topMotor.setControl(coastRequest);
        } else {
            topMotor.setControl(velocityVoltage.withVelocity(manipTargetRPM / 60));
        }
    }
}