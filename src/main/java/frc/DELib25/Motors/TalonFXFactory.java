// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.DELib25.Util.PhoneixUtil;

/** Add your docs here. */
public class TalonFXFactory {
    public static TalonFX createTalonFX(MotorConstants motorConstants, boolean useDefaultConfig){
        TalonFX talon = createTalonFX(motorConstants.id, motorConstants.bus);
        if(useDefaultConfig) PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(getDefaultConfig(motorConstants))); 
        return talon;
    }

    public static TalonFX createTalonFX(MotorConstants motorConstants, TalonFXConfiguration configuration){
        TalonFX talon = createTalonFX(motorConstants.id, motorConstants.bus);
        PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(configuration));
        return talon;
    }

    public static TalonFX createSlaveTalon(MotorConstants slave, int masterId, boolean opposeMasterDirection){
        TalonFX talon = createTalonFX(slave, true);
        PhoneixUtil.checkErrorAndRetry(() -> talon.setControl(new Follower(masterId, opposeMasterDirection)));
        return talon;
    }

    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        config.Feedback.SensorToMechanismRatio = 1.0;

        config.Audio.BeepOnBoot = true;

        return config;
    }

        public static TalonFXConfiguration getDefaultConfig(MotorConstants motorConstants) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = MotorConstants.toNeturalMode(motorConstants.isBrake);
        config.MotorOutput.Inverted = MotorConstants.toInvertedType(motorConstants.CounterClockwisePositive);
        config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        config.Feedback.SensorToMechanismRatio = 1.0;

        config.Audio.BeepOnBoot = true;

        return config;
    }

    private static TalonFX createTalonFX(int id, String bus){
        TalonFX talon = new TalonFX(id, bus);
        talon.clearStickyFaults();
        return talon;
    }
}