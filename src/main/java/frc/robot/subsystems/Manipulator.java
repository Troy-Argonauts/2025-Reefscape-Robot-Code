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
 * Class representing the Manipulator Subsystem.
 * 
 * @author VedNakum, firearcher2012, ASH-will-WIN, Evan13019, shaquilleinoatmeal.
 */
public class Manipulator extends SubsystemBase {
    private DoubleLogEntry topMotorSupplyCurrentLog;
    private DoubleLogEntry bottomMotorSupplyCurrentLog;
    private DoubleLogEntry lateratorMotorSupplyCurrentLog;

    private double lateratorCurrentPosition; 
    private double lateratorTarget;

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private TalonFX lateratorMotor;
    
    private DigitalInput lateratorLimit;

    private DigitalInput funnelBeamBreak;
    private DigitalInput manipulatorBeamBreak;

    private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Slot0Configs slot0Config = config.Slot0;

    /**
     * Constructor for the Manipulator class. Initializes motors, sensors, and configurations.
     * 
     * <ul>
     *  <li>Instantiates motors and limit switch.</li>
     *  <li>Assigns CAN IDs and Sensor Slots. </li>
     *  <li>Sets current limits, Nuetral Modes, and Inversion direction for motors.</li>
     *  <li>Sets Bottom Manipulator Motor to follow Top Manipulator Motor</li>
     *  <li>Creates data log objects.  </li>
     * </ul>
     */
    public Manipulator() {
        TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration lateratorConfiguration = new TalonFXConfiguration();

        topMotor = new TalonFX(Constants.Manipulator.TOP_MOTOR_CAN_ID);
        bottomMotor = new TalonFX(Constants.Manipulator.BOTTOM_MOTOR_CAN_ID);
        lateratorMotor = new TalonFX(Constants.Manipulator.LATERATOR_MOTOR_CAN_ID);

        lateratorLimit = new DigitalInput(Constants.Manipulator.LATERATOR_LIMIT_SWITCH);

        funnelBeamBreak = new DigitalInput(Constants.Manipulator.FUNNEL_BEAM_BREAK);
        manipulatorBeamBreak = new DigitalInput(Constants.Manipulator.MANIPULATOR_BEAM_BREAK);

        topConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        lateratorConfiguration.CurrentLimits.SupplyCurrentLimit = 20;

        topConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        lateratorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        topMotor.getConfigurator().apply(topConfiguration);
        bottomMotor.getConfigurator().apply(bottomConfiguration);
        lateratorMotor.getConfigurator().apply(lateratorConfiguration);

        bottomMotor.setControl(new Follower(Constants.Manipulator.TOP_MOTOR_CAN_ID, false));
        lateratorMotor.getConfigurator().apply(slot0Config);

        DataLog log = DataLogManager.getLog();

        topMotorSupplyCurrentLog = new DoubleLogEntry((log), "Top Manipulator Motor Supply Current");
        bottomMotorSupplyCurrentLog = new DoubleLogEntry(log, "Bottom Manipulator Motor Supply Current");
        lateratorMotorSupplyCurrentLog = new DoubleLogEntry(log, "Laterator Motor Supply Current");

    }

     /**
     * Outputs values to SmartDashboard; Appends logs with corresponding data.
     */
    @Override
    public void periodic() {

        topMotorSupplyCurrentLog.append(topMotor.getSupplyCurrent().getValueAsDouble());
        bottomMotorSupplyCurrentLog.append(bottomMotor.getSupplyCurrent().getValueAsDouble());   
        lateratorMotorSupplyCurrentLog.append(lateratorMotor.getSupplyCurrent().getValueAsDouble()); 

        SmartDashboard.putBoolean("Coral Ready", isCoralReady());
        SmartDashboard.putNumber("Lat Supply Current", lateratorMotor.getSupplyCurrent().getValueAsDouble());

        // lateratorMotor.setControl(positionVoltage.withPosition(lateratorTarget));

        // if (getLateratorLimit() == true) {
        //     resetLateratorEncoder();
        // }

        
    }

    /**
     * Represents the different states of the Manipulator
     */
    public enum ManipulatorStates {
        IN,

        INTAKE,

        SCORING,

        OFF;
    }

    /**
     * Represents the different states of the Laterator.
     */
    public enum LateratorStates {
        /**
         * Turns off power to Laterator motor. Used to retract Laterator.
         */
        OFF,
        /**
         * Apply power to Laterator motor to hold it in place.
         */
        HOLD,
        /**
         * Apply power to Laterator Motor to move it outward.
         */
        OUT;
    }

    /**
     * Applies power to Laterator Motor based on given state.
     * @param state The desired {@code LateratorStates} enumerator
     */
    public void setLateratorState(LateratorStates state) {
        switch (state) {
            case OUT: 
                setLateratorRawPower(0.3);
                break;
            case OFF: 
                setLateratorRawPower(0);
                break;
            case HOLD:
                setLateratorRawPower(0.3);
                break;
            
        }
    }

 

    /**
     * Retrieves current status of the Laterator’s limit switch.
     * @return True if limit is detected, false if not
     */
    public boolean getLateratorLimit(){
        return lateratorLimit.get();
    }

    /**
     * Retrieves current status of Manipulator Beam Brake sensor. If active, a Coral is within the Manipulator.
     * @return true if Coral is detected, false if not
     */
    public boolean isCoralReady() {
        return !manipulatorBeamBreak.get();
    }

 

    /**
     * Sets the Laterator motors to a percentage of available voltage.
     * @param power The desired percentage value between -1 and 1
     */
    public void setLateratorRawPower(double power) {
        lateratorMotor.set(power);
    }
 
    /**
     * Applies power to Manipulator motors based on given state.
     * @param state The desired {@code ManipulatorStates} enumerator
     */
    public void setManipState(ManipulatorStates state) {
        switch (state) {
            case IN: 
                setManipulatorRawPower(-0.3); 
                break;
            case INTAKE: 
                setManipulatorRawPower(0.15);
                break;
            case SCORING:
                setManipulatorRawPower(0.15);
                break;
            case OFF:
                setManipulatorRawPower(0);
                break;
            
        }
    }

    /**
     * Sets the Manipulator motors to a percentage of available voltage.
     * @param speed The desired percentage value between -1 and 1
     */
    public void setManipulatorRawPower(double speed){
        topMotor.set(speed);
    }

 

    /**
     * Returns whether a coral has entered the funnel
     * @return whether coral has entered funnel
     */
    public boolean hasCoralEntered() {
        return funnelBeamBreak.get();
    }
}