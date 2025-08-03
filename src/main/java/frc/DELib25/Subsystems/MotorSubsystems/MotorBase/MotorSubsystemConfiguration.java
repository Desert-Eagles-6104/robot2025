package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.DELib25.Motors.MotorConfiguration;
import frc.DELib25.Util.ProjectConstants;
import frc.DELib25.Motors.PIDContainer;

public class MotorSubsystemConfiguration {

    /** The name of this subsystem. */
    public String subsystemName = "";

    /** Master motor configuration (required). */
    public MotorConfiguration master = null;

    /** Slave motor configurations (optional). */
    public MotorConfiguration[] slaves = null;

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

    public double allowableError = 6.0;

    public double homePosition = 0.0;

    public double angleOffset = 0.0;

    // Cached configurations
    private TalonFXConfiguration masterTalonFXConfiguration;
    private TalonFXConfiguration[] slavesTalonFXConfiguration;

    /**
     * Returns the lazily generated master motor TalonFXConfiguration.
     */
    public TalonFXConfiguration getMasterTalonFXConfiguration() {
        if (master == null) {
            throw new IllegalStateException("Master motor configuration is required but was null.");
        }
        if (masterTalonFXConfiguration == null) {
            masterTalonFXConfiguration = createTalonFXConfiguration(this, master);
        }
        return masterTalonFXConfiguration;
    }

    /**
     * Returns the TalonFXConfiguration for the given slave index.
     * 
     * @param slaveIndex index of the slave motor
     */
    public TalonFXConfiguration getSlaveTalonFXConfiguration(int slaveIndex) {
        if (slaves == null || slaveIndex < 0 || slaveIndex >= slaves.length) {
            throw new IndexOutOfBoundsException("Invalid slave index: " + slaveIndex);
        }

        if (slavesTalonFXConfiguration == null) {
            slavesTalonFXConfiguration = new TalonFXConfiguration[slaves.length];
        }

        if (slavesTalonFXConfiguration[slaveIndex] == null) {
            MotorConfiguration slaveConfig = slaves[slaveIndex];
            if (slaveConfig == null) {
                throw new IllegalStateException("Slave motor at index " + slaveIndex + " is null.");
            }
            slavesTalonFXConfiguration[slaveIndex] = createTalonFXConfiguration(this, slaveConfig);
        }

        return slavesTalonFXConfiguration[slaveIndex];
    }

    /**
     * Internal method to create a fully configured TalonFXConfiguration based
     * on this config and a motor.
     */
    private static TalonFXConfiguration createTalonFXConfiguration(MotorSubsystemConfiguration cfg, MotorConfiguration motor) {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(cfg.motionMagicCruiseVelocity * cfg.rotationsPerPositionUnit)
            .withMotionMagicAcceleration(cfg.motionMagicAcceleration * cfg.rotationsPerPositionUnit)
            .withMotionMagicJerk(cfg.motionMagicJerk * cfg.rotationsPerPositionUnit);

        talonConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(cfg.forwardSoftLimit != ProjectConstants.ERROR_CODE)
            .withForwardSoftLimitThreshold(cfg.forwardSoftLimit * cfg.rotationsPerPositionUnit)
            .withReverseSoftLimitEnable(cfg.reverseSoftLimit != ProjectConstants.ERROR_CODE)
            .withReverseSoftLimitThreshold(cfg.reverseSoftLimit * cfg.rotationsPerPositionUnit);

        talonConfig.CurrentLimits
            .withStatorCurrentLimitEnable(cfg.enableStatorCurrentLimit)
            .withStatorCurrentLimit(cfg.statorCurrentLimit)
            .withSupplyCurrentLimitEnable(cfg.enableSupplyCurrentLimit)
            .withSupplyCurrentLimit(cfg.supplyCurrentLimit);

        talonConfig.withSlot0(PIDContainer.toSlot0Configs(cfg.pidContainerSlot0));
        talonConfig.withSlot1(PIDContainer.toSlot1Configs(cfg.pidContainerSlot1));

        talonConfig.MotorOutput
            .withInverted(MotorConfiguration.toInvertedType(motor.CounterClockwisePositive))
            .withNeutralMode(MotorConfiguration.toNeutralMode(motor.isBrake));

        // Sensor ratio
        talonConfig.Feedback.withSensorToMechanismRatio(cfg.sensorToMechanismRatio);

        return talonConfig;
    }
}
