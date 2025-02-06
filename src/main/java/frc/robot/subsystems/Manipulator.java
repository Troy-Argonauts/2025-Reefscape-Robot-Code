package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Manipulator() {
        topMotor = new TalonFX(0);
        bottomMotor = new TalonFX(0);
        lateratorMotor = new TalonFX(0);

        limitSwitch = new DigitalInput(0);

        topMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);
        lateratorMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();
        
        bottomMotor.setControl(new Follower(0, true));

        topConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 30;

        topMotor.getConfigurator().apply(topConfiguration);
        bottomMotor.getConfigurator().apply(bottomConfiguration);

        DataLog log = DataLogManager.getLog();

        currentManipulatorVelocityLog = new DoubleLogEntry((log), "Current Manipulator Velocity");
        topMotorSupplyCurrentLog = new DoubleLogEntry((log), "Top Motor Supply Current");
    
    }

    @Override
    public void periodic() {

    currentManipulatorVelocityLog.append(currentManipulatorVelocity);
    topMotorSupplyCurrentLog.append(topMotor.getSupplyCurrent().getValueAsDouble());

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


    // Evan is cool
    public double getVelocity() {
        return (topMotor.getVelocity().getValueAsDouble() + bottomMotor.getVelocity().getValueAsDouble()) / 2;
    } 
    public double getManipulatorDesiredVelocity() {
        return desiredManipulatorVelocity;
    }
}