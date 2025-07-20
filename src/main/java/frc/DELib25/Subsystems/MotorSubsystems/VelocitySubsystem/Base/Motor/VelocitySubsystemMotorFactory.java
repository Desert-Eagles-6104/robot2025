// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Base.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController.AccelStrategy;
import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
// import frc.DELib25.Motors.SparkFlexFactory;
// import frc.DELib25.Motors.SparkMaxFactory;
import frc.DELib25.Motors.TalonFXFactory;
import frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.VelocitySubsystemConfiguration;

/** Add your docs here. */
public class VelocitySubsystemMotorFactory {
    public static TalonFX createTalonFX(VelocitySubsystemConfiguration configuration){
        TalonFXConfiguration talonConfiguration = TalonFXFactory.getDefaultConfig();
        talonConfiguration.MotionMagic
        .withMotionMagicCruiseVelocity(configuration.motionMagicCruiseVelocity * configuration.rotationsPerPositionUnit)
        .withMotionMagicAcceleration(configuration.motionMagicAcceleration * configuration.rotationsPerPositionUnit)
        .withMotionMagicJerk(configuration.motionMagicJerk * configuration.rotationsPerPositionUnit);

        talonConfiguration.CurrentLimits
        .withStatorCurrentLimitEnable(configuration.enableStatorCurrentLimit)
        .withStatorCurrentLimit(configuration.statorCurrentLimit)
        .withSupplyCurrentLimitEnable(configuration.enableSupplyCurrentLimit)
        .withSupplyCurrentLimit(configuration.supplyCurrentLimit);

        talonConfiguration.withSlot0(PIDContainer.toSlot0Configs(configuration.pidContainer));

        talonConfiguration.MotorOutput
        .withInverted(MotorConstants.toInvertedType(configuration.master.CounterClockwisePositive))
        .withNeutralMode(MotorConstants.toNeturalMode(configuration.master.isBrake));

        talonConfiguration.Feedback.withSensorToMechanismRatio(configuration.sensorToMechanismRatio);

        TalonFX talon = TalonFXFactory.createTalonFX(configuration.master, talonConfiguration);
        return talon;
    }

    public static TalonFX[] createSlaveTalonFX(VelocitySubsystemConfiguration configuration){
        if(configuration.slaves != null){
            TalonFX[] slaveFX = new TalonFX[configuration.slaves.length];
            for (int i = 0; i < configuration.slaves.length; i++) {
                MotorConstants slaveConstants = configuration.slaves[i];
                slaveFX[i] = TalonFXFactory.createSlaveTalon(slaveConstants, configuration.master.id, configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
                slaveFX[i].optimizeBusUtilization();
            }
            return slaveFX;
        }
        return null;
    }

    // public static CANSparkFlex createSparkFlex(VelocitySubsystemConfiguration configuration){
    //     CANSparkFlex spark = SparkFlexFactory.createSparkFlex(configuration.master, true);

    //     spark.getEncoder().setPositionConversionFactor(configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit);
    //     spark.getEncoder().setVelocityConversionFactor((configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit) /60.0);

    //     PIDContainer.toSparkPIDController(configuration.pidContainer, spark);

    //     spark.getPIDController().setSmartMotionMaxAccel(configuration.motionMagicAcceleration, 0);
    //     spark.getPIDController().setSmartMotionMaxVelocity(configuration.motionMagicCruiseVelocity, 0);
    //     spark.getPIDController().setSmartMotionAllowedClosedLoopError(configuration.allowableError, 0);
    //     spark.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    //     spark.setSmartCurrentLimit(configuration.supplyCurrentLimit);
    //     return spark;
    // }

    // public static CANSparkFlex[] createSlaveSparkFlex(VelocitySubsystemConfiguration configuration, CANSparkFlex master){
    //     CANSparkFlex[] SparkFlexSlave = new CANSparkFlex[configuration.slaves.length];
    //     for (int i = 0; i < configuration.slaves.length; i++) {
    //         MotorConstants slaveConstants = configuration.slaves[i];
    //         SparkFlexSlave[i] = SparkFlexFactory.createSlaveSparkFlex(master, slaveConstants.id, configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
    //     }
    //     return SparkFlexSlave;
    // }

    // public static CANSparkMax createSparkMax(VelocitySubsystemConfiguration configuration){
    //     CANSparkMax spark = SparkMaxFactory.createSparkMax(configuration.master, true);

    //     spark.getEncoder().setPositionConversionFactor(configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit);
    //     spark.getEncoder().setVelocityConversionFactor((configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit) /60.0);

    //     PIDContainer.toSparkPIDController(configuration.pidContainer, spark);

    //     spark.getPIDController().setSmartMotionMaxAccel(configuration.motionMagicAcceleration, 0);
    //     spark.getPIDController().setSmartMotionMaxVelocity(configuration.motionMagicCruiseVelocity, 0);
    //     spark.getPIDController().setSmartMotionAllowedClosedLoopError(configuration.allowableError, 0);
    //     spark.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    //     spark.setSmartCurrentLimit(configuration.supplyCurrentLimit);
    //     return spark;
    // }

    // public static CANSparkMax[] createSlaveSparkMax(VelocitySubsystemConfiguration configuration , CANSparkMax master){
    //    CANSparkMax[] SparkMaxSlave = new CANSparkMax[configuration.slaves.length];
    //     for (int i = 0; i < configuration.slaves.length; i++) {
    //         MotorConstants slaveConstants = configuration.slaves[i];
    //         SparkMaxSlave[i] = SparkMaxFactory.createSlaveSparkMax(master, slaveConstants.id, configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
    //     }
    //     return SparkMaxSlave;
    // }

    // public static CANSparkMax[] createSlaveSparkFlex(VelocitySubsystemConfiguration configuration, CANSparkMax master){
    //     CANSparkMax[] SparkMaxSlave = new CANSparkMax[configuration.slaves.length];
    //     for (int i = 0; i < configuration.slaves.length; i++) {
    //         MotorConstants slaveConstants = configuration.slaves[i];
    //         SparkMaxSlave[i] = SparkMaxFactory.createSlaveSparkMax(master, slaveConstants.id, configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
    //     }
    //     return SparkMaxSlave;
    // }

    
}