// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Motors;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class MotorConstants {
    public int id = -1;
    public String bus = "";
    public boolean CounterClockwisePositive = false;
    public boolean isBrake = false;

    public MotorConstants(int id, String bus, boolean CounterClockwisePositive, boolean isBrake){
        this.id = id;
        this.bus = bus;
        this.CounterClockwisePositive = CounterClockwisePositive;
        this.isBrake = isBrake;
    }

    /**
     * for talonFX
     * @param CounterClockwisePositive
     * @return
     */
    public static InvertedValue toInvertedType(boolean CounterClockwisePositive){
        if (CounterClockwisePositive) {
            return InvertedValue.CounterClockwise_Positive;
        }
        return InvertedValue.Clockwise_Positive;
    }

    /**
     * for TalonFX
     * @param isBrake
     * @return
     */
    public static NeutralModeValue toNeturalMode(boolean isBrake){
        if (isBrake) {
            return NeutralModeValue.Brake;
        }
        return NeutralModeValue.Coast;
    }

    public static IdleMode toIdleMode(boolean isBrake){
        if (isBrake) {
            return IdleMode.kBrake;
        }
        return IdleMode.kCoast;
    }
}
