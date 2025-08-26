package frc.DELib25.Tuning;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PidCalibrations extends SubsystemBase {
    private final TalonFX motor;
    private double kP = 0.0, kI = 0.0, kD = 0.0;
    private boolean lastButtonState = false;
    private int changesNum = 0;
    private GenericEntry targetEntry, targetPosEntry, kPEntry, kIEntry, kDEntry;
    private double lastKP = Double.NaN, lastKI = Double.NaN, lastKD = Double.NaN;
    private final PositionDutyCycle posReq = new PositionDutyCycle(0).withEnableFOC(true);
    private double target = 0.0;
    private double targetPos = 5.0;
    private boolean targetToggle = false;
    private final String tabName;

    public PidCalibrations(int motorID) {
        this(new TalonFX(motorID));
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        cfg.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40);
        this.motor.getConfigurator().apply(cfg);
    }

    public PidCalibrations(TalonFX motor) {
        this.motor = motor;
        this.motor.getConfigurator().apply(new Slot0Configs());
        motor.setPosition(0.0);
        this.tabName = "PID-" + motor.getDeviceID();
        var tab = Shuffleboard.getTab(this.tabName);
        tab.addDouble("Target Now", () -> this.target);
        targetEntry = tab.add("Go to Target", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        targetPosEntry = tab.add("Target Pos", targetPos)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 50, "block increment", 5))
            .getEntry();
        kPEntry = tab.add("kP", kP)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 50, "block increment", 0.1))
            .getEntry();
        kIEntry = tab.add("kI", kI)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10, "block increment", 0.01))
            .getEntry();
        kDEntry = tab.add("kD", kD)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 0.3, "block increment", 0.05))
            .getEntry();
    }

    @Override
    public void periodic() {
        double newKP = kPEntry.getDouble(kP);
        double newKI = kIEntry.getDouble(kI);
        double newKD = kDEntry.getDouble(kD);
        double newTargetPos = targetPosEntry.getDouble(targetPos);

        boolean gainsChanged = (newKP != lastKP) || (newKI != lastKI) || (newKD != lastKD);
        if (gainsChanged) {
            kP = newKP; kI = newKI; kD = newKD;
            motor.getConfigurator().apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
            lastKP = kP; lastKI = kI; lastKD = kD;
            SmartDashboard.putString(tabName + "/ApplyStatus", "Gains changed! " + (++changesNum));
        }

        boolean buttonState = targetEntry.getBoolean(false);
        if (newTargetPos != targetPos) {
            targetPos = newTargetPos;
            SmartDashboard.putNumber(tabName + "/NewTargetPos", targetPos);
            if (Math.abs(motor.getPosition().getValueAsDouble() - target) < 0.02) {
                target = targetToggle ? targetPos : 0.0;
            }
        }
        if (buttonState != lastButtonState) {
            targetToggle = buttonState;
            target = targetToggle ? targetPos : 0.0;
        }
        lastButtonState = buttonState;

        motor.setControl(posReq.withPosition(target));
        SmartDashboard.putNumber(tabName + "/Pos(rot)", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(tabName + "/Target(rot)", target);
        SmartDashboard.putNumber(tabName + "/DutyOut", motor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putBoolean(tabName + "/AtTarget(~0.02)", Math.abs(motor.getPosition().getValueAsDouble() - target) < 0.02);
    }
}
