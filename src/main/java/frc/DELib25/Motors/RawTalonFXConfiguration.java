package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.DELib25.Util.ProjectConstants;
/**
 * This calss should be used for storing a raw TalonFXConfiguration to be converted to a TalonFXConfiguration
 * its relly just for double braces initialization.
 * It shold (and will) be phased out by next season.
 */
public class RawTalonFXConfiguration {
    /**
     * Ratio between motor rotations and position units (e.g., height of an
     * elevator).
     */
    public double rotationsPerPositionUnit = 1.0;

    /** Ratio between sensor units and mechanism movement. */
    public double sensorToMechanismRatio = 1.0;

    public PIDContainer pidContainerSlot0 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public PIDContainer pidContainerSlot1 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public double motionMagicCruiseVelocity = 0.0;
    public double motionMagicAcceleration = 0.0;
    public double motionMagicJerk = 0.0;

    public int supplyCurrentLimit = 60;
    public boolean enableSupplyCurrentLimit = false;
    public int statorCurrentLimit = 40;
    public boolean enableStatorCurrentLimit = false;

    /** Software limits. Use ERROR_CODE to disable. */
    public double forwardSoftLimit = ProjectConstants.ERROR_CODE;
    public double reverseSoftLimit = ProjectConstants.ERROR_CODE;

    /**
     * Internal method to create a fully configured TalonFXConfiguration based
     * on this config and a motor.
     */
    public static TalonFXConfiguration createTalonFXConfiguration(RawTalonFXConfiguration constants) {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        // all of these "with" methods are just glorified setters
        talonConfig.MotionMagic
                .withMotionMagicCruiseVelocity(constants.motionMagicCruiseVelocity * constants.rotationsPerPositionUnit)
                .withMotionMagicAcceleration(constants.motionMagicAcceleration * constants.rotationsPerPositionUnit)
                .withMotionMagicJerk(constants.motionMagicJerk * constants.rotationsPerPositionUnit);

        talonConfig.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(constants.forwardSoftLimit != ProjectConstants.ERROR_CODE)
                .withForwardSoftLimitThreshold(constants.forwardSoftLimit * constants.rotationsPerPositionUnit)
                .withReverseSoftLimitEnable(constants.reverseSoftLimit != ProjectConstants.ERROR_CODE)
                .withReverseSoftLimitThreshold(constants.reverseSoftLimit * constants.rotationsPerPositionUnit);


        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        talonConfig.CurrentLimits
                .withStatorCurrentLimitEnable(constants.enableStatorCurrentLimit)
                .withStatorCurrentLimit(constants.statorCurrentLimit)
                .withSupplyCurrentLimitEnable(constants.enableSupplyCurrentLimit)
                .withSupplyCurrentLimit(constants.supplyCurrentLimit);

        talonConfig.withSlot0(PIDContainer.toSlot0Configs(constants.pidContainerSlot0));
        talonConfig.withSlot1(PIDContainer.toSlot1Configs(constants.pidContainerSlot1));

        talonConfig.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        talonConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        talonConfig.MotorOutput.PeakReverseDutyCycle = -1.0;

        // Sensor ratio
        talonConfig.Feedback.withSensorToMechanismRatio(constants.sensorToMechanismRatio);

        talonConfig.Audio.withBeepOnBoot(true);

        return talonConfig;
    }
    public static TalonFXConfiguration defaultConfig() {
        return createTalonFXConfiguration(new RawTalonFXConfiguration());
    }
    
}
