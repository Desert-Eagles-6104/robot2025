// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemConfiguration;
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
  // TODO: Nothing uses this 
  public final static class Vision{
    public static final CameraSettings aprilTagCameraSettings = new CameraSettings(0.30821, 0, 0.10689, 0, 15.13, 0, false); 
    public static final CameraSettings gamePieceCameraSettings = new CameraSettings(0, 0, 0, 0, 0, 0, false); 

    public static final double cameraHeight = 0.10689;//TODO: This must be tuned to specific robot
    public static final double tragetHeight = 0.307975;//height of april tag center from the floor - reef april tag
    public static final double cameraPitch = 15.13;//limelight 3 cameraPitch

  }

  public final class Elevator {
    public static final MotorSubsystemConfiguration ElevatorConfiguration = new MotorSubsystemConfiguration(){{
     
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
      forwardSoftLimit = 0.70;

      reverseSoftLimit = 0.0;
      //#endregion soft limits

      allowableError = 0.8;

      homePosition = 0.0;
    }};
  }

  public final class GripperArm {
    public static final MotorSubsystemConfiguration configuration = new MotorSubsystemConfiguration() {
      {
        subsystemName = "gripperArm";

        master = new MotorConstants(1, "rio", true, true);

        rotationsPerPositionUnit = 1.0 / 360.0;

        sensorToMechanismRatio = 2.28;

        pidContainerSlot0 = new PIDContainer(0, 0, 0, 0.75, 160, 0, 0.65, GravityTypeValue.Arm_Cosine);

        pidContainerSlot1 = new PIDContainer(0, 0, 0, 0.75, 160, 0, 0.65, GravityTypeValue.Arm_Cosine);

        //#region motion magic values
        motionMagicCruiseVelocity = 35000;

        motionMagicAcceleration = 30000;

        motionMagicJerk = 360;
        //#endregion motion magic values

        //#region cuurent limit
        supplyCurrentLimit = 55;

        enableSupplyCurrentLimit = true;

        statorCurrentLimit = 35;

        enableStatorCurrentLimit = false;
        //#endregion current limit

        //#region soft limits
        forwardSoftLimit = 36.5;

        reverseSoftLimit = -89.5;

        //#endregion soft limits

        allowableError = 3.5;

        homePosition = -89.0;

        angleOffset = 0.163;
      }
    };
  }

  // TODO: Nothing uses this
  public final class IntakeArm {
    public static final MotorSubsystemConfiguration configuration = new MotorSubsystemConfiguration(){{

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
    //Integration
    public static final double TimeToDropIntegraion = 0.5;
    }

  public static final class Gripper{
      public static final int motorId = 2;
      public static final int beamBreakPort = 2;
      public static final double intakeOutput = 0.6;
      public static final double OutTakeOutput = 0.7;
      public static final double Ks = 0.0; 
      public static final double Kv = 0.0; 
      public static final double Ka = 0.0;
      public static final double Kp = 0.0; 
      public static final double Ki = 0.0; 
      public static final double Kd = 0.0;
      public static final double supplyCurrentLimit = 20;
      public static final boolean SupplyCurrentLimitEnable = true;
      public static final int SensorToMechanismRatio = 3;
      public static final double DutyCycleNeutralDeadband = 1.0;
      public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
      public static final int frequencyHz = 0;
    }

  public static final SysidConfiguration sysidConfiguration = new SysidConfiguration() {
    {
      /** The voltage ramp rate used for quasistatic test routines. */
      m_rampRate = Volts.of(4).div(Seconds.of(1));

      /** The step voltage output used for dynamic test routines. */
      m_stepVoltage = Volts.of(3);

      /** Safety timeout for the test routine commands. */
      m_timeout = Seconds.of(2);
      //#endregion mechanisem
    }
  };
}
