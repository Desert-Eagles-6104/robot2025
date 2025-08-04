package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    public double rotationsPerPositionUnit = 1.0;


    public MotorConstants(int id, String bus, boolean counterClockwisePositive, boolean isBrake) {
        this(id, bus, RawTalonFXConfiguration.defaultConfig(), counterClockwisePositive, isBrake);
    }

    public MotorConstants(int id, String bus, RawTalonFXConfiguration config, boolean counterClockwisePositive, boolean isBrake) {
        this(id, bus, RawTalonFXConfiguration.createTalonFXConfiguration(config), counterClockwisePositive, isBrake);
    }

    public MotorConstants(int id, String bus, TalonFXConfiguration talonFXConfiguration, boolean counterClockwisePositive, boolean isBrake) {
        this.id = id;
        this.bus = bus;

        this.talonFXConfiguration = talonFXConfiguration;
        this.talonFXConfiguration.MotorOutput.NeutralMode = MotorConstants.toNeutralMode(isBrake);
        this.talonFXConfiguration.MotorOutput.Inverted = MotorConstants.toInvertedType(counterClockwisePositive);

        this.counterClockwisePositive = counterClockwisePositive;
        this.isBrake = isBrake;
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
    

}
