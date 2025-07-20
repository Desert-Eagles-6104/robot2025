// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.Base.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.SoftLimitDirection;
// import com.revrobotics.SparkPIDController.AccelStrategy;
import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
// import frc.DELib25.Motors.SparkFlexFactory;
// import frc.DELib25.Motors.SparkMaxFactory;
import frc.DELib25.Motors.TalonFXFactory;
import frc.DELib25.Subsystems.MotorSubsystems.ServoSubsystemConfiguration;

/** Add your docs here. */
public class ServoSubsystemMotorFactory {
    public static TalonFX createTalonFX(ServoSubsystemConfiguration configuration){
        TalonFXConfiguration talonConfiguration = TalonFXFactory.getDefaultConfig();
        talonConfiguration.MotionMagic
        .withMotionMagicCruiseVelocity(configuration.motionMagicCruiseVelocity * configuration.rotationsPerPositionUnit)
        .withMotionMagicAcceleration(configuration.motionMagicAcceleration * configuration.rotationsPerPositionUnit)
        .withMotionMagicJerk(configuration.motionMagicJerk * configuration.rotationsPerPositionUnit);

        talonConfiguration.SoftwareLimitSwitch
        .withForwardSoftLimitEnable(configuration.forwardSoftLimit != -99999)
        .withForwardSoftLimitThreshold(configuration.forwardSoftLimit * configuration.rotationsPerPositionUnit)
        .withReverseSoftLimitEnable(configuration.reverseSoftLimit != -99999)
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

    public static TalonFX[] createSlaveTalonFX(ServoSubsystemConfiguration configuration){
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

    // public static CANSparkFlex createSparkFlex(ServoSubsystemConfiguration configuration){
    //     CANSparkFlex spark = SparkFlexFactory.createSparkFlex(configuration.master, true);

    //     spark.getEncoder().setPositionConversionFactor(configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit);
    //     spark.getEncoder().setVelocityConversionFactor((configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit) /60.0);

    //     PIDContainer.toSparkPIDController(configuration.pidContainerSlot0, spark);

    //     spark.getPIDController().setSmartMotionMaxAccel(configuration.motionMagicAcceleration, 0);
    //     spark.getPIDController().setSmartMotionMaxVelocity(configuration.motionMagicCruiseVelocity, 0);
    //     spark.getPIDController().setSmartMotionAllowedClosedLoopError(configuration.allowableError, 0);
    //     spark.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    //     spark.enableSoftLimit(SoftLimitDirection.kForward, configuration.forwardSoftLimit != -99999);
    //     spark.setSoftLimit(SoftLimitDirection.kForward, (float) configuration.forwardSoftLimit);
    //     spark.enableSoftLimit(SoftLimitDirection.kReverse, configuration.reverseSoftLimit != -99999);
    //     spark.setSoftLimit(SoftLimitDirection.kReverse, (float) configuration.reverseSoftLimit);

    //     spark.setSmartCurrentLimit(configuration.supplyCurrentLimit);
    //     return spark;
    // }

    // public static CANSparkFlex[] createSlaveSparkFlex(ServoSubsystemConfiguration configuration, CANSparkFlex master){
    //     CANSparkFlex[] SparkFlexSlave = new CANSparkFlex[configuration.slaves.length];
    //     for (int i = 0; i < configuration.slaves.length; i++) {
    //         MotorConstants slaveConstants = configuration.slaves[i];
    //         SparkFlexSlave[i] = SparkFlexFactory.createSlaveSparkFlex(master, slaveConstants.id, configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
    //     }
    //     return SparkFlexSlave;
    // }

    // public static CANSparkMax createSparkMax(ServoSubsystemConfiguration configuration){
    //     CANSparkMax spark = SparkMaxFactory.createSparkMax(configuration.master, true);

    //     spark.getEncoder().setPositionConversionFactor(configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit);
    //     spark.getEncoder().setVelocityConversionFactor((configuration.sensorToMechanismRatio * configuration.rotationsPerPositionUnit) /60.0);

    //     PIDContainer.toSparkPIDController(configuration.pidContainerSlot0, spark);

    //     spark.getPIDController().setSmartMotionMaxAccel(configuration.motionMagicAcceleration, 0);
    //     spark.getPIDController().setSmartMotionMaxVelocity(configuration.motionMagicCruiseVelocity, 0);
    //     spark.getPIDController().setSmartMotionAllowedClosedLoopError(configuration.allowableError, 0);
    //     spark.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    //     spark.enableSoftLimit(SoftLimitDirection.kForward, configuration.forwardSoftLimit != -99999);
    //     spark.setSoftLimit(SoftLimitDirection.kForward, (float) configuration.forwardSoftLimit);
    //     spark.enableSoftLimit(SoftLimitDirection.kReverse, configuration.reverseSoftLimit != -99999);
    //     spark.setSoftLimit(SoftLimitDirection.kReverse, (float) configuration.reverseSoftLimit);

    //     spark.setSmartCurrentLimit(configuration.supplyCurrentLimit);
    //     return spark;
    // }

    // public static CANSparkMax[] createSlaveSparkFlex(ServoSubsystemConfiguration configuration, CANSparkMax master){
    //     CANSparkMax[] SparkMaxSlave = new CANSparkMax[configuration.slaves.length];
    //     for (int i = 0; i < configuration.slaves.length; i++) {
    //         MotorConstants slaveConstants = configuration.slaves[i];
    //         SparkMaxSlave[i] = SparkMaxFactory.createSlaveSparkMax(master, slaveConstants.id, configuration.master.CounterClockwisePositive != slaveConstants.CounterClockwisePositive);
    //     }
    //     return SparkMaxSlave;
    // }

}