// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Motors.MotorType;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.Swerve.SwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveModuleConstants;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import frc.DELib25.Sysid.SysidConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  public final static class Vision{
    public static final CameraSettings aprilTagCameraSettings = new CameraSettings(0.30821, 0, 0.10689, 0, 15.13, 0, false); 
    public static final CameraSettings gamePieceCameraSettings = new CameraSettings(0, 0, 0, 0, 0, 0, false); 

    public static final double cameraHeight = 0.10689;//TODO: This must be tuned to specific robot
    public static final double tragetHeight = 0.307975;//high of april tag center from floor - reef april tag
    public static final double cameraPitch = 15.13;//liemlight 3 cameraPitch

  }

  public final static class Swerve{
    public static final double x = 0.75 /2.0; //width/2 
    public static final double y = 0.75 /2.0; //length/2

    public static SwerveConstants swerveConstants = new SwerveConstants(){{
      chosenModule =  //TODO: This must be tuned to specific robot
      COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

      /*String bus */
      String canBus = "Canivore";

      /* Drivetrain Constants */
      wheelCircumference = chosenModule.wheelCircumference;

      /*swerve module position*/
      frontLeftPos = new Translation2d(x,y);
      modulesPositions[0] = frontLeftPos;
      frontRightPos = new Translation2d(x,-y);
      modulesPositions[1] = frontRightPos;
      backLeftPos = new Translation2d(-x,y);
      modulesPositions[2] = backLeftPos;
      backRightPos = new Translation2d(-x,-y);
      modulesPositions[3] = backRightPos;

      /* Module Gear Ratios */
      driveGearRatio = chosenModule.driveGearRatio;
      angleGearRatio = chosenModule.angleGearRatio;

      /* Motor Inverts */
      angleMotorInvert = chosenModule.angleMotorInvert;
      driveMotorInvert = chosenModule.driveMotorInvert;

      /* Angle Encoder Invert */
      canCoderInvert = chosenModule.cancoderInvert;

      /*Feedback Sensor Azimuth */
      feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


      /* Swerve Current Limiting */
      angleContinuousCurrentLimit = 25;
      anglePeakCurrentLimit = 40;
      anglePeakCurrentDuration = 0.1;
      angleEnableCurrentLimit = true;

      driveContinuousCurrentLimit = 40;
      driveStatorCurrentLimit = 60;
      drivePeakCurrentLimit = 60;
      drivePeakCurrentDuration = 0.1;
      driveEnableCurrentLimit = true;

      /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
      * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
      openLoopRamp = 0.2;
      closedLoopRamp = 0.2;

      /* Angle Motor PID Values */
      angleKP = chosenModule.angleKP;
      angleKI = chosenModule.angleKI;
      angleKD = chosenModule.angleKD;

      /* Drive Motor PID Values */
      driveKP = 3.5; //TODO: This must be tuned to specific robot (sysid bratan)
      driveKI = 0.0;
      driveKD = 0.0;
      driveKS = 1.2;
      driveKV = 0.0;

      /* Heading PID Values */
      HeadingKP = 0.5;
      HeadingKI = 0.0;
      HeadingKD = 0.0;
      HeadingTolerence = 1;


      /* Drive Motor Characterization Values 
      * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
      driveKS = 0; //TODO: This must be tuned to specific robot
      driveKV = 0;
      driveKA = 0;

      /*wheel parameters */
      WheelRadius = 2;
      WheelCircumference = WheelRadius * 2 * Math.PI;

      /* Swerve Profiling Values */
      /** Meters per Second */
      maxSpeed = 4.9; //TODO: This must be tuned to specific robot
      /** Radians per Second */
      maxAngularVelocity = 5.21 / 0.31992 * 0.9; //Robot linear max speed divided by the robot radius 

      /* Neutral Modes */
      angleNeutralMode = NeutralMode.Brake;
      driveNeutralMode = NeutralMode.Brake;


      FL = new SwerveModuleConstants(10, 11, 12, Rotation2d.fromDegrees(0.0), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), frontLeftPos);
      FR = new SwerveModuleConstants(20, 21, 22, Rotation2d.fromDegrees(0.0), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), frontRightPos);
      BL = new SwerveModuleConstants(30, 31, 32, Rotation2d.fromDegrees(0.0), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), backLeftPos);
      BR = new SwerveModuleConstants(40, 41, 42, Rotation2d.fromDegrees(0.0), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), backRightPos);

      filepath = "/home/lvuser/natinst/ModuleOffsets.csv";
  }};
}



  public final class Elevator {
    public static final ServoSubsystemConfiguration ElevatorConfiguration = new ServoSubsystemConfiguration(){{
     
      motorType = MotorType.talonFX;

      subsystemName = "Elevator";

      slaves = new MotorConstants[]{new MotorConstants(50,"rio",true,true)};

      master = new MotorConstants(51,"rio",true ,true);

      rotationsPerPositionUnit = 1.0 / (0.0363728 * Math.PI);

      sensorToMechanismRatio = 10.2857142857;
      
      pidContainerSlot0 = new PIDContainer(0, 1.2733, 0.060729, 0.334, 0.15, 0.0, 0.0);

      // pidContainerSlot1 = new PIDContainer(0.0, 0.0, 1.0, 0.15, 1000.0, 0.0, 0.0);
      //#region motion magic values
      motionMagicCruiseVelocity = 3;
      
      motionMagicAcceleration = 6;

      motionMagicJerk = 9;
      //#endregion motion magic values

      //#region cuurent limit
      supplyCurrentLimit = 40;

      enableSupplyCurrentLimit = true;

      statorCurrentLimit = 40;

      enableStatorCurrentLimit = true;
      //#endregion current limit

      //#region soft limits 
      forwardSoftLimit = 0.65;

      reverseSoftLimit = 0.0;
      //#endregion sofr limits

      allowableError = 0.05;

      homePosition = 0.0;
    }};
  }

  public final class GripperArm {
    public static final ServoSubsystemConfiguration configuration = new ServoSubsystemConfiguration(){{
      motorType = MotorType.talonFX;

      subsystemName = "gripperArm";

      master = new MotorConstants(1,"rio",true ,true);

      rotationsPerPositionUnit = 1.0/360.0;

      sensorToMechanismRatio = 2.28;
      
      pidContainerSlot0 = new PIDContainer(0, 0, 0, 0.75, 75, 0, 0.5, GravityTypeValue.Arm_Cosine);

      pidContainerSlot1 = new PIDContainer(0, 0, 0, 0.75, 75, 0, 0.5, GravityTypeValue.Arm_Cosine);

      //#region motion magic values
      motionMagicCruiseVelocity = 10000;
      
      motionMagicAcceleration = 20000;

      motionMagicJerk = 20;
      //#endregion motion magic values

      //#region cuurent limit
      supplyCurrentLimit = 60; 

      enableSupplyCurrentLimit = true;

      statorCurrentLimit = 40;

      enableStatorCurrentLimit = false;
      //#endregion current limit

      //#region soft limits 
      forwardSoftLimit = 37;

      reverseSoftLimit = -90;
      //#endregion sofr limits

      allowableError = 1.0;

      homePosition = 37.0;
    }};
  }

  public final class IntakeArm {
    public static final ServoSubsystemConfiguration configuration = new ServoSubsystemConfiguration(){{
      motorType = MotorType.talonFX;

      subsystemName = "intakeArm";

      // slaves = new MotorConstants[]{new MotorConstants(51,"rio",true,true)};

      master = new MotorConstants(4,"rio",true ,true);
      
      rotationsPerPositionUnit = 1.0/360.0;

      sensorToMechanismRatio = 20;
      
      pidContainerSlot0 = new PIDContainer(0, 0.0, 0.0, 0.65, 400.0, 0.0, 0.0);

      pidContainerSlot1 = new PIDContainer(0, 0.0, 0.0, 0.65, 3, 0.0, 0.0);
      //#region motion magic values
      motionMagicCruiseVelocity = 10000;
      
      motionMagicAcceleration = 15000;

      motionMagicJerk = 20000;
      //#endregion motion magic values

      //#region cuurent limit
      supplyCurrentLimit = 60; 

      enableSupplyCurrentLimit = true;

      statorCurrentLimit = 40;

      enableStatorCurrentLimit = false;
      //#endregion current limit

      //#region soft limits 
      forwardSoftLimit = 105.0;

      reverseSoftLimit = 0.0;
      //#endregion sofr limits

      allowableError = 2.0;

      homePosition = 0;
    }};
  }

  public static final class Intake{
    public static final int motorId = 3;
    public static final int beamBreakPort = 0;
    public static final double intakeOutput = 0.6;
    public static final double OutTakeOutput = 0.7;
    public static final double Ks = 0.0; 
    public static final double Kv = 0.0; 
    public static final double Ka = 0.0;
    public static final double Kp = 0.0; 
    public static final double Ki = 0.0; 
    public static final double Kd = 0.0;
    public static final double supplyCurrentLimit = 0;
    public static final boolean SupplyCurrentLimitEnable = true;
    public static final int SensorToMechanismRatio = 0;
    public static final double DutyCycleNeutralDeadband = 0.0;
    public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
    public static final int frequencyHz = 0;
    }



  public static final class Gripper{
      public static final int motorId = 2;
      public static final int beamBreakPort = 0;
      public static final double intakeOutput = 0.6;
      public static final double OutTakeOutput = 0.7;
      public static final double Ks = 0.0; 
      public static final double Kv = 0.0; 
      public static final double Ka = 0.0;
      public static final double Kp = 0.0; 
      public static final double Ki = 0.0; 
      public static final double Kd = 0.0;
      public static final double supplyCurrentLimit = 0.0;
      public static final boolean SupplyCurrentLimitEnable = true;
      public static final int SensorToMechanismRatio = 0;
      public static final double DutyCycleNeutralDeadband = 0.0;
      public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
      public static final int frequencyHz = 0;
    }

  public static final SysidConfiguration sysidConfiguration = new SysidConfiguration(){{
    /** The voltage ramp rate used for quasistatic test routines. */
    m_rampRate = Volts.of(4).div(Seconds.of(1));

    /** The step voltage output used for dynamic test routines. */
    m_stepVoltage = Volts.of(3);

    /** Safety timeout for the test routine commands. */
    m_timeout = Seconds.of(2);
    //#endregion mechanisem
  }};
}
