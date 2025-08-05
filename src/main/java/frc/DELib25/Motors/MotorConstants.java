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

    // TalonFX configuration for this motor
    private TalonFXConfiguration talonFXConfiguration;

    /**
     * Ratio between motor rotations and position units (e.g., height of an
     * elevator).
     */
    public double rotationsPerPositionUnit;



    public MotorConstants(int id, String bus, boolean counterClockwisePositive, double rotationsPerPositionUnit, TalonFXConfiguration talonFXConfiguration) {
        this.id = id;
        this.bus = bus;

        this.talonFXConfiguration = talonFXConfiguration;
        this.talonFXConfiguration.MotorOutput.Inverted = MotorConstants.toInvertedType(counterClockwisePositive);

        this.rotationsPerPositionUnit = rotationsPerPositionUnit;
        this.counterClockwisePositive = counterClockwisePositive;
    }

    public TalonFXConfiguration getTalonFXConfig() {
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

    public static IdleMode toIdleMode(boolean isBrake) {
        if (isBrake) {
            return IdleMode.kBrake;
        }
        return IdleMode.kCoast;
    }

    public static TalonFXConfiguration createTalonFXConfiguration(
        boolean isBrake,
        double rotationsPerPositionUnit,
        double sensorToMechanismRatio,
        PIDContainer[] pidContainerSlots,// Its in arr just so i dont have to make a now method for each variation of slots
        double motionMagicCruiseVelocity,
        double motionMagicAcceleration,
        double motionMagicJerk,
        int supplyCurrentLimit,
        boolean enableSupplyCurrentLimit,
        int statorCurrentLimit,
        boolean enableStatorCurrentLimit,
        double forwardSoftLimit,
        double reverseSoftLimit
    ) {
        TalonFXConfiguration talonConfig = createSlaveTalonFXConfiguration(isBrake, supplyCurrentLimit, enableSupplyCurrentLimit, statorCurrentLimit, enableStatorCurrentLimit);
        // all of these "with" methods are just glorified setters
        talonConfig.MotionMagic
                .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity * rotationsPerPositionUnit)
                .withMotionMagicAcceleration(motionMagicAcceleration * rotationsPerPositionUnit)
                .withMotionMagicJerk(motionMagicJerk * rotationsPerPositionUnit);

        talonConfig.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(forwardSoftLimit != ProjectConstants.ERROR_CODE)
                .withForwardSoftLimitThreshold(forwardSoftLimit * rotationsPerPositionUnit)
                .withReverseSoftLimitEnable(reverseSoftLimit != ProjectConstants.ERROR_CODE)
                .withReverseSoftLimitThreshold(reverseSoftLimit * rotationsPerPositionUnit);

        talonConfig.withSlot0(PIDContainer.toSlot0Configs(pidContainerSlots[0]));
        if (pidContainerSlots.length > 1)
            talonConfig.withSlot1(PIDContainer.toSlot1Configs(pidContainerSlots[1]));


        // Sensor ratio
        talonConfig.Feedback.withSensorToMechanismRatio(sensorToMechanismRatio);


        return talonConfig;
    }
    
    /*
     * This method creates a TalonFXConfiguration for a slave TalonFX motor.
     * It can also be used for a base configuration of a master TalonFX motor.
     */
    public static TalonFXConfiguration createSlaveTalonFXConfiguration(
        boolean isBrake,
        int supplyCurrentLimit,
        boolean enableSupplyCurrentLimit,
        int statorCurrentLimit,
        boolean enableStatorCurrentLimit
    ) {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        talonConfig.MotorOutput.withNeutralMode(isBrake? NeutralModeValue.Brake : NeutralModeValue.Coast);

        talonConfig.MotorOutput
            .withDutyCycleNeutralDeadband(0.01)
            .withPeakForwardDutyCycle(1.0)
            .withPeakReverseDutyCycle(-1.0);

        talonConfig.CurrentLimits
            .withSupplyCurrentLowerLimit(60)
            .withSupplyCurrentLowerTime(0.1)
            
            .withStatorCurrentLimitEnable(enableStatorCurrentLimit)
            .withStatorCurrentLimit(statorCurrentLimit)
            .withSupplyCurrentLimitEnable(enableSupplyCurrentLimit)
            .withSupplyCurrentLimit(supplyCurrentLimit);

        talonConfig.Audio.withBeepOnBoot(true);

        return talonConfig;
    }

}
