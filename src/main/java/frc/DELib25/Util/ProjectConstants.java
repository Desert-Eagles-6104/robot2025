package frc.DELib25.Util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Subsystems.Swerve.SwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveModuleConstants;

public class ProjectConstants {
    public static final double ERROR_CODE = -6104;
    public final static class Swerve{
        public static final double x = 0.75 /2.0; //width/2 
        public static final double y = 0.75 /2.0; //length/2

        public static COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);//TODO: This must be tuned to specific robot

        public static final TalonFXConfiguration driveTalonFXConfigs = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(chosenModule.driveMotorInvert)
                    .withNeutralMode(NeutralModeValue.Brake)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(chosenModule.driveGearRatio)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLowerLimit(60)
                    .withSupplyCurrentLowerTime(0.1)
            ).withSlot0(/* Drive Motor Characterization Values 
            * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
                PIDContainer.toSlot0Configs(new PIDContainer(0, 0, 0, 0, 3,0, 0))
            ).withOpenLoopRamps(/* These values are used by the drive falcon to ramp in open loop and closed loop driving.
            * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(0.2)
                    .withVoltageOpenLoopRampPeriod(0.2)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(0.2)
                    .withVoltageClosedLoopRampPeriod(0.2)
            );
        
            public static final TalonFXConfiguration angleTalonFXConfigs = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(chosenModule.angleMotorInvert)
                    .withNeutralMode(NeutralModeValue.Coast)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(chosenModule.angleGearRatio)
            ).withClosedLoopGeneral(
                new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(true)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLowerTime(0.01)
            ).withSlot0(
                new Slot0Configs()
                    .withKP(chosenModule.angleKP)
                    .withKI(chosenModule.angleKI)
                    .withKD(chosenModule.angleKD)
            );

        public static SwerveConstants swerveConstants = new SwerveConstants(driveTalonFXConfigs, angleTalonFXConfigs) {
            {
                /*String bus */
                canBus = "Canivore";

                /* Drivetrain Constants */
                wheelCircumference = chosenModule.wheelCircumference;

                /* Angle Encoder Invert */
                canCoderInvert = chosenModule.cancoderInvert;

                /*Feedback Sensor Azimuth */
                feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


                //openLoopRamp = 0.2;
                //closedLoopRamp = 0.2;
                /* Heading PID Values */
                HeadingKP = 0.5;
                HeadingKI = 0.0;
                HeadingKD = 0.0;
                HeadingTolerence = 1.5;

                /*wheel parameters */
                WheelRadius = 2;
                WheelCircumference = WheelRadius * 2 * Math.PI;

                /* Swerve Profiling Values */
                /** Meters per Second */
                maxSpeed = 4.9; //TODO: This must be tuned to specific robot
                /** Radians per Second */
                maxAngularVelocity = 5.21 / 0.31992 * 0.9; //Robot linear max speed divided by the robot radius 
                //TODO: update  the module offsets 
                FL = new SwerveModuleConstants(
                    10, 11 , 12, Rotation2d.fromRotations(0.110840), new Translation2d(x, y)
                    ); 
                FR = new SwerveModuleConstants(
                    20, 21, 22, Rotation2d.fromRotations(0.265869),     new Translation2d(x, -y)
                    );
                BL = new SwerveModuleConstants(
                    30, 31, 32, Rotation2d.fromRotations(-0.269043), new Translation2d(-x, y)
                    ); 
                BR = new SwerveModuleConstants(
                    40, 41, 42, Rotation2d.fromRotations(-0.158936), new Translation2d(-x, -y)
                    );
            }
        };
    }
}
