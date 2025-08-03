package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.DELib25.Util.ProjectConstants;

public class MotorConstants {
    public final int id;
    public final String bus;
    public final boolean counterClockwisePositive;
    public final boolean isBrake;

    // TalonFX configuration for this motor
    private TalonFXConfiguration talonFXConfiguration;

    /**
     * Ratio between motor rotations and position units (e.g., height of an
     * elevator).
     */
    public final double rotationsPerPositionUnit = 1.0;

    /** Ratio between sensor units and mechanism movement. */
    public final double sensorToMechanismRatio = 1.0;

    public final PIDContainer pidContainerSlot0 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public final PIDContainer pidContainerSlot1 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public final double motionMagicCruiseVelocity = 0.0;
    public final double motionMagicAcceleration = 0.0;
    public final double motionMagicJerk = 0.0;

    public final int supplyCurrentLimit = 60;
    public final boolean enableSupplyCurrentLimit = false;
    public final int statorCurrentLimit = 40;
    public final boolean enableStatorCurrentLimit = false;

    /** Software limits. Use ERROR_CODE to disable. */
    public final double forwardSoftLimit = ProjectConstants.ERROR_CODE;
    public final double reverseSoftLimit = ProjectConstants.ERROR_CODE;

    public MotorConstants(int id, String bus, boolean counterClockwisePositive, boolean isBrake) {
        this.id = id;
        this.bus = bus;
        this.counterClockwisePositive = counterClockwisePositive;
        this.isBrake = isBrake;
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        if (this.talonFXConfiguration == null) {
            this.talonFXConfiguration = createTalonFXConfiguration(this);
        }
        return this.talonFXConfiguration;
    }

    /**
     * for talonFX
     * 
     * @param counterClockwisePositive
     * @return
     */
    public static InvertedValue toInvertedType(boolean counterClockwisePositive) {
        if (counterClockwisePositive) {
            return InvertedValue.CounterClockwise_Positive;
        }
        return InvertedValue.Clockwise_Positive;
    }

    /**
     * for TalonFX
     * 
     * @param isBrake
     * @return
     */
    public static NeutralModeValue toNeutralMode(boolean isBrake) {
        if (isBrake) {
            return NeutralModeValue.Brake;
        }
        return NeutralModeValue.Coast;
    }

    public static IdleMode toIdleMode(boolean isBrake) {
        if (isBrake) {
            return IdleMode.kBrake;
        }
        return IdleMode.kCoast;
    }

    /**
     * Internal method to create a fully configured TalonFXConfiguration based
     * on this config and a motor.
     */
    private static TalonFXConfiguration createTalonFXConfiguration(MotorConstants constants) {
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

        talonConfig.CurrentLimits
                .withStatorCurrentLimitEnable(constants.enableStatorCurrentLimit)
                .withStatorCurrentLimit(constants.statorCurrentLimit)
                .withSupplyCurrentLimitEnable(constants.enableSupplyCurrentLimit)
                .withSupplyCurrentLimit(constants.supplyCurrentLimit);

        talonConfig.withSlot0(PIDContainer.toSlot0Configs(constants.pidContainerSlot0));
        talonConfig.withSlot1(PIDContainer.toSlot1Configs(constants.pidContainerSlot1));

        talonConfig.MotorOutput
                .withInverted(MotorConstants.toInvertedType(constants.counterClockwisePositive))
                .withNeutralMode(MotorConstants.toNeutralMode(constants.isBrake));

        // Sensor ratio
        talonConfig.Feedback.withSensorToMechanismRatio(constants.sensorToMechanismRatio);

        return talonConfig;
    }
}
