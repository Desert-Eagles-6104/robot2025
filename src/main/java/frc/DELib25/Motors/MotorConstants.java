package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorConstants {
    public final int id;
    public final String bus;
    public final boolean counterClockwisePositive;

    // TalonFX configuration for this motor
    private final TalonFXConfiguration talonFXConfiguration;

    /**
     * Ratio between motor rotations and position units (e.g., height of an
     * elevator).
     */
    public double rotationsPerPositionUnit;

    public MotorConstants(int id, String bus, boolean counterClockwisePositive, double rotationsPerPositionUnit, TalonFXConfiguration talonFXConfiguration){
        this.id = id;
        this.bus = bus;
        this.counterClockwisePositive = counterClockwisePositive;
        this.rotationsPerPositionUnit = rotationsPerPositionUnit;
        this.talonFXConfiguration = talonFXConfiguration;
        this.talonFXConfiguration.MotorOutput.Inverted = MotorConstants.toInvertedType(counterClockwisePositive);
    }

    public MotorConstants(int id, String bus, boolean counterClockwisePositive, TalonFXConfiguration talonFXConfiguration){
        this.id = id;
        this.bus = bus;
        this.counterClockwisePositive = counterClockwisePositive;
        this.talonFXConfiguration = talonFXConfiguration;
        this.talonFXConfiguration.MotorOutput.Inverted = MotorConstants.toInvertedType(counterClockwisePositive);
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

}
