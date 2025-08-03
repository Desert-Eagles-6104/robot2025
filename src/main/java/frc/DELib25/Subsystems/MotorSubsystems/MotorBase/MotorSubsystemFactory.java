package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Util.PhoneixUtil;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class MotorSubsystemFactory {
    public static TalonFX createTalonFX(MotorConstants motorConstants) {
        TalonFX talon = createTalonFX(motorConstants.id, motorConstants.bus);
        PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(getDefaultConfig(motorConstants)));
        return talon;
    }

    public static TalonFX createTalonFX(MotorConstants motorConstants, TalonFXConfiguration configuration) {
        TalonFX talon = createTalonFX(motorConstants.id, motorConstants.bus);
        PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(configuration));
        return talon;
    }

    public static TalonFX createSlaveTalon(MotorConstants slave, int masterId, boolean opposeMasterDirection) {
        TalonFX talon = createTalonFX(slave);
        PhoneixUtil.checkErrorAndRetry(() -> talon.setControl(new Follower(masterId, opposeMasterDirection)));
        return talon;
    }

    private static TalonFXConfiguration baseDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Audio.BeepOnBoot = true;

        return config;
    }

    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = baseDefaultConfig();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        return config;
    }

    public static TalonFXConfiguration getDefaultConfig(MotorConstants motorConstants) {
        TalonFXConfiguration config = baseDefaultConfig();
        config.MotorOutput.NeutralMode = MotorConstants.toNeutralMode(motorConstants.isBrake);
        config.MotorOutput.Inverted = MotorConstants.toInvertedType(motorConstants.counterClockwisePositive);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;
    }

    private static TalonFX createTalonFX(int id, String bus) {
        TalonFX talon = new TalonFX(id, bus);
        talon.clearStickyFaults();
        return talon;
    }

    public static TalonFX createTalonFX(MotorSubsystemConfiguration configuration) {
        TalonFX talon = createTalonFX(configuration.master, configuration.master.getTalonFXConfiguration());
        return talon;
    }

    public static TalonFX[] createSlaveTalonsFX(MotorSubsystemConfiguration configuration) {
        if (configuration.slaves != null) {
            TalonFX[] slaveFX = new TalonFX[configuration.slaves.length];
            for (int i = 0; i < configuration.slaves.length; i++) {
                MotorConstants slaveConstants = configuration.slaves[i];
                slaveFX[i] = createSlaveTalon(slaveConstants, configuration.master.id,
                        configuration.master.counterClockwisePositive != slaveConstants.counterClockwisePositive);
                slaveFX[i].optimizeBusUtilization();
            }
            return slaveFX;
        }
        return null;
    }
}
