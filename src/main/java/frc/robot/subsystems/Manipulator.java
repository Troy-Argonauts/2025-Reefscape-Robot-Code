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
 * 
 * @author VedNakum, firearcher2012, ASH-will-WIN, Evan13019, shaquilleinoatmeal.
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
    
    private DigitalInput lateratorLimit;

    private DigitalInput funnel;
    private DigitalInput manipulatorA;
    private DigitalInput manipulatorB;

    private ManipulatorStates manipulatorState = ManipulatorStates.OFF;

    private final CoastOut coastRequest = new CoastOut();

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Slot0Configs slot0Config = config.Slot0;

    /**
     * Constructor for the Manipulator class. Initializes motors, sensors, and configurations.
     */
    public Manipulator() {
        topMotor = new TalonFX(Constants.Manipulator.TOP_MOTOR_CAN_ID);
        bottomMotor = new TalonFX(Constants.Manipulator.BOTTOM_MOTOR_CAN_ID);
        lateratorMotor = new TalonFX(Constants.Manipulator.LATERATOR_MOTOR_CAN_ID);

        lateratorLimit = new DigitalInput(Constants.Manipulator.LATERATOR_LIMIT_SWITCH);

        funnel = new DigitalInput(Constants.Manipulator.FUNNEL_BEAM_BREAK);
        manipulatorA = new DigitalInput(Constants.Manipulator.MANIPULATOR_BEAM_BREAK_A);
        manipulatorB = new DigitalInput(Constants.Manipulator.MANIPULATOR_BEAM_BREAK_B);

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

        DataLog log = DataLogManager.getLog();

        currentManipRPMLog = new DoubleLogEntry((log), "Current Manipulator RPM");
        topMotorSupplyCurrentLog = new DoubleLogEntry((log), "Top Motor Supply Current");
        bottomMotorSupplyCurrentLog = new DoubleLogEntry(log, "Bottom Motor Supply Current");
        lateratorMotorSupplyCurrentLog = new DoubleLogEntry(log, "Laterator Motor Supply Current");

    }

     /**
     * This method is called periodically by the scheduler. Logs current values and updates 
     * SmartDashboard with current and target RPMs.
     */
    @Override
    public void periodic() {
        currentManipRPM = (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble()) / (2 * 60);

        currentManipRPMLog.append(currentManipRPM);
        topMotorSupplyCurrentLog.append(topMotor.getSupplyCurrent().getValueAsDouble());
        bottomMotorSupplyCurrentLog.append(bottomMotor.getSupplyCurrent().getValueAsDouble());   
        lateratorMotorSupplyCurrentLog.append(lateratorMotor.getSupplyCurrent().getValueAsDouble()); 

        SmartDashboard.putNumber("Current Roller Velocity", currentManipRPM);
        SmartDashboard.putNumber("Desired Roller Velocity", manipTargetRPM);

        topMotor.setControl(velocityVoltage.withVelocity(manipTargetRPM / 60));

        if (getLateratorLimit() == true) {
            resetLateratorEncoder();
        }
    }
    /**
     * States of the Manipulator (IN, OUT, OFF)
     */
    public enum ManipulatorStates {
        IN(-800),

        OUT(800),

        OFF(0);

        final double manipulatorVelocity;

        ManipulatorStates(double manipulatorVelocity) {
            this.manipulatorVelocity = manipulatorVelocity;
        }

    }
    /**
     * States of the Laterator (IN, OUT, OFF)
     */
    public enum LateratorStates {
        IN,

        OUT,

        OFF;
    }

    /**
     * Sets the laterator's state until it reaches the max laterator position if out or until it reaches 0 if in
     * @param state The desired state of the laterator
     */
    public void setLateratorState(LateratorStates state) {
        switch (state) {
            case IN: 
                setLateratorRawPower(-0.3);
            case OUT: 
                setLateratorRawPower(0.3);
            case OFF:
                setLateratorRawPower(0);
        }
    }

    /**
     * Checks if laterator encoder has reached the max laterator position
     * @return True if encoder hits max laterator position, false if not
     */
    public boolean lateratorExtended() {
        if (lateratorMotor.getPosition().getValueAsDouble() >= Constants.Manipulator.MAX_LATERATOR_POSITION) {
            return true;
        }
        return false;
    }

    /**
     * Checks if the top PID loop has finished.
     * @return True if the top PID loop has finished, false otherwise.
     */
    public boolean isTopPidFinished() {
        return (Math.abs(topMotor.getPosition().getValueAsDouble()) <= 50);
    }

    /**
     * Checks if the limit switch is pressed.
     * @return True if the limit switch is pressed, false otherwise.
     */
    public boolean getLateratorLimit(){
        return lateratorLimit.get();
    }

    /**
     * Checks if the funnel beam break is detected.
     * @return True if the funnel beam break is detected, false otherwise.
     */
    public boolean getFunnelBeamBreak() {
        return funnel.get();
    }

    /**
     * Checks if the manipulator A beam break is detected.
     * @return True if the manipulator A beam break is detected, false otherwise.
     */
    public boolean getManipBeamBreakA() {
        return manipulatorA.get();
    }

    /**
     * Checks if the manipulator B beam break is detected.
     * @return True if the manipulator B beam break is detected, false otherwise.
     */
    public boolean getManipBeamBreakB() {
        return manipulatorB.get();
    }


    /**
     * Returns true if beam break A or B is being triggered
     * @return true if either beam break A or B is true, false if both beam breaks are false
     */
    public boolean isCoralReady() {
        if (getManipBeamBreakA() == true || getManipBeamBreakB() == true) {
            return true;
        }
        return false;
    }


    // Evan is cool***!!!
    /**
     * Gets the current velocity of the manipulator.
     * @return The current velocity of the manipulator in RPM.
     */
    public double getAverageRPM() {
        return (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble()) / (2 * 60);
    } 

    /**
     * Gets the top motor's current RPM
     * @return Top motor's current RPM
     */
    public double getTopRPM() {
        return (topMotor.getVelocity().getValueAsDouble() * 60);
    }

    /**
     * Gets bottom motor's current RPM
     * @return Bottom motor's current RPM
     */
    public double getBottomRPM() {
        return (bottomMotor.getVelocity().getValueAsDouble() * 60);
    } 

    /**
     * Gets the desired velocity of the manipulator.
     * @return The desired velocity of the manipulator in RPM.
     */
    public double getManipTarget() {
        return manipTargetRPM;
    }

    /**
     * Gets the current position of the laterator.
     * @return The current position of the laterator.
     */
    public double getCurrentLateratorPosition() {
        return lateratorCurrentPosition;
    }

    /**
     * Sets the raw power for the laterator motor.
     * @param power The power to set for the laterator motor.
     */
    public void setLateratorRawPower(double power) {
        lateratorMotor.set(power);
    }


    /**
     * Sets the manipulator motors desired target
     * @param target the target to set for the manipulator motor 
     */
    public void setManipDesiredTarget(double target) {
        manipTargetRPM = target;
    }

    /**
     * Sets the manipulator motors state (Expecting RPM)
     * @param state the state to set for the manipulator motor
     */
    public void setManipState(ManipulatorStates state) {
        manipTargetRPM = state.manipulatorVelocity;
        manipulatorState = state;
    }

    /**
     * Sets laterator raw power to 0.3 until hits max position
     */
    public void lateratorIN() {
        while (getCurrentLateratorPosition() < Constants.Manipulator.MAX_LATERATOR_POSITION) {
            setLateratorRawPower(0.3);
        }
    }

    /**
     * sets laterator raw power to -0.3 until hits 0
     */
    public void lateratorOUT() {
        while (getCurrentLateratorPosition() > 0) {
            setLateratorRawPower(-0.3);
        }
    }


    /**
     * resets laterator motor encoder
     */
    public void resetLateratorEncoder() {
        lateratorMotor.setPosition(0);
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