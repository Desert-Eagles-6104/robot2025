package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Motors.TalonFXFactory;
import frc.DELib25.Util.ProjectConstants;

public class MotorSubsystemFactory {

    public static TalonFX createTalonFX(MotorSubsystemConfiguration configuration) {
        TalonFXConfiguration talonConfiguration = TalonFXFactory.getDefaultConfig();
        talonConfiguration.MotionMagic
            .withMotionMagicCruiseVelocity(configuration.motionMagicCruiseVelocity * configuration.rotationsPerPositionUnit)
            .withMotionMagicAcceleration(configuration.motionMagicAcceleration * configuration.rotationsPerPositionUnit)
            .withMotionMagicJerk(configuration.motionMagicJerk * configuration.rotationsPerPositionUnit);

        talonConfiguration.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(configuration.forwardSoftLimit != ProjectConstants.ERROR_CODE)
            .withForwardSoftLimitThreshold(configuration.forwardSoftLimit * configuration.rotationsPerPositionUnit)
            .withReverseSoftLimitEnable(configuration.reverseSoftLimit != ProjectConstants.ERROR_CODE)
            .withReverseSoftLimitThreshold(configuration.reverseSoftLimit * configuration.rotationsPerPositionUnit);

        talonConfiguration.CurrentLimits
            .withStatorCurrentLimitEnable(configuration.enableStatorCurrentLimit)
            .withStatorCurrentLimit(configuration.statorCurrentLimit)
            .withSupplyCurrentLimitEnable(configuration.enableSupplyCurrentLimit)
            .withSupplyCurrentLimit(configuration.supplyCurrentLimit);

        talonConfiguration.withSlot0(PIDContainer.toSlot0Configs(configuration.pidContainerSlot0));
        talonConfiguration.withSlot1(PIDContainer.toSlot1Configs(configuration.pidContainerSlot1));

        talonConfiguration.MotorOutput
            .withInverted(MotorConstants.toInvertedType(configuration.master.CounterClockwisePositive))
            .withNeutralMode(MotorConstants.toNeturalMode(configuration.master.isBrake));

        talonConfiguration.Feedback.withSensorToMechanismRatio(configuration.sensorToMechanismRatio);
        
        TalonFX talon = TalonFXFactory.createTalonFX(configuration.master, talonConfiguration);
        return talon;
    }
    
    public static TalonFX[] createSlaveTalonFX(MotorSubsystemConfiguration configuration) {
        if (configuration.slaves != null) {
            TalonFX[] slaveFX = new TalonFX[configuration.slaves.length];
            for (int i = 0; i < configuration.slaves.length; i++) {
                MotorConstants slaveConstants = configuration.slaves[i];
                slaveFX[i] = TalonFXFactory.createSlaveTalon(slaveConstants, configuration.master.id,
                        configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
                slaveFX[i].optimizeBusUtilization();
            }
            return slaveFX;
        }
        return null;
    }
}
