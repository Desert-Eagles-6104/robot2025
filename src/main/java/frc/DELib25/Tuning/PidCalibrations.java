package frc.DELib25.Tuning;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PidCalibrations extends SubsystemBase {
    private final TalonFX motors[];
    private double kP = 0.0, kI = 0.0, kD = 0.0;
    private GenericEntry kPEntry, kIEntry, kDEntry;
    private double lastKP = Double.NaN, lastKI = Double.NaN, lastKD = Double.NaN;
    private final String tabName;
    private boolean slidersEnabled = false;

    public PidCalibrations(int motorID, boolean slidersEnabled) {
        this(new TalonFX(motorID), slidersEnabled);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        cfg.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40);
        
        for (TalonFX motor : this.motors) {
            motor.getConfigurator().apply(cfg);
        }
    }

    public PidCalibrations(TalonFX motor, boolean slidersEnabled) {
        this(new TalonFX[] { motor }, slidersEnabled);
    }

    public PidCalibrations(TalonFX[] motors, boolean slidersEnabled) {
        this.motors = motors;
        this.slidersEnabled = slidersEnabled;
        this.applyPidValues();//We want a fresh application of the values.
        String name = "PID-";
        for (TalonFX motor : motors) {
            name += motor.getDeviceID() + "-";
        }
        this.tabName = name.substring(0, name.length() - 1);
        this.setEntriess();
    }

    private void setEntriess() {
        var tab = Shuffleboard.getTab(this.tabName);
        if (this.slidersEnabled) {
            this.kPEntry = tab.add("kP", this.kP)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 50, "block increment", 0.1))
                .getEntry();
            this.kIEntry = tab.add("kI", this.kI)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 10, "block increment", 0.01))
                .getEntry();
            this.kDEntry = tab.add("kD", this.kD)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 0.3, "block increment", 0.05))
                .getEntry();
        } else {
            this.kPEntry = tab.add("kP", this.kP).getEntry();
            this.kIEntry = tab.add("kI", this.kI).getEntry();
            this.kDEntry = tab.add("kD", this.kD).getEntry();
        }
    }

    @Override
    public void periodic() {
        double newKP = this.kPEntry.getDouble(this.kP);
        double newKI = this.kIEntry.getDouble(this.kI);
        double newKD = this.kDEntry.getDouble(this.kD);

        boolean gainsChanged = (newKP != this.lastKP) || (newKI != this.lastKI) || (newKD != this.lastKD);
        if (gainsChanged) {
            this.kP = newKP; this.kI = newKI; this.kD = newKD;
            this.applyPidValues();
            this.lastKP = this.kP; this.lastKI = this.kI; this.lastKD = this.kD;
        }

    }

    private void applyPidValues(){
        for (TalonFX motor : this.motors) {
            motor.getConfigurator().apply(new Slot0Configs().withKP(this.kP).withKI(this.kI).withKD(this.kD));
        }
    }
}
