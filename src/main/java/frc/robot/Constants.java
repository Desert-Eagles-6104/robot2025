// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    public static final CameraSettings aprilTagCameraSettings = new CameraSettings(0.30821, 0, 0.10689, 0, 15.13, 0, false); //TODO: This must be tuned to specific robot
    public static final CameraSettings gamePieceCameraSettings = new CameraSettings(0, 0, 0, 0, 0, 0, false); //TODO: This must be tuned to specific robot

    public static final double cameraHeight = 0.10689;//TODO: This must be tuned to specific robot
    public static final double tragetHeight = 1.435;//TODO: This must be tuned to specific robot
    public static final double cameraPitch = 15.13;//TODO: This must be tuned to specific robot
  }

  public final static class Swerve{
    public static final double x = 0.4853 /2.0; //width/2 //TODO: This must be tuned to specific robot
    public static final double y = 0.425 /2.0; //length/2//TODO: This must be tuned to specific robot

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

      driveStatorCurrentLimit = 60;
      driveContinuousCurrentLimit = 40;
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
      WheelRadius = 0.0508;
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

  public static final SysidConfiguration sysidConfiguration = new SysidConfiguration(){{
    /** The voltage ramp rate used for quasistatic test routines. */
    m_rampRate = Volts.of(1).div(Seconds.of(1));

    /** The step voltage output used for dynamic test routines. */
    m_stepVoltage = Volts.of(4);

    /** Safety timeout for the test routine commands. */
    m_timeout = Seconds.of(3.5);
    //#endregion mechanisem
  }};
}
