package frc.DELib25.Util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib25.Subsystems.Swerve.SwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveModuleConstants;

public class ProjectConstants {
    public static final double ERROR_CODE = -6104;
    public final static class Swerve{
        public static final double x = 0.75 /2.0; //width/2 
        public static final double y = 0.75 /2.0; //length/2

        public static SwerveConstants swerveConstants = new SwerveConstants() {
            {
                // TODO: This must be tuned to specific robot
                chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

                /*String bus */
                String canBus = "Canivore";

                /* Drivetrain Constants */
                wheelCircumference = chosenModule.wheelCircumference;

                /*swerve module position*/
                frontLeftPos = new Translation2d(x, y);
                modulesPositions[0] = frontLeftPos;
                frontRightPos = new Translation2d(x, -y);
                modulesPositions[1] = frontRightPos;
                backLeftPos = new Translation2d(-x, y);
                modulesPositions[2] = backLeftPos;
                backRightPos = new Translation2d(-x, -y);
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

                // double [] angleOffset= {0.105957, 0.260498, -0.268066,-0.159424};

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
                driveKP = 3.0; //TODO: This must be tuned to specific robot (sysid bratan)
                driveKI = 0.0;
                driveKD = 0.0;
                driveKS = 1.2;
                driveKV = 0.0;

                /* Heading PID Values */
                HeadingKP = 0.5;
                HeadingKI = 0.0;
                HeadingKD = 0.0;
                HeadingTolerence = 1.5;

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
                angleNeutralMode = NeutralMode.Coast;
                driveNeutralMode = NeutralMode.Brake;

                FL = new SwerveModuleConstants(10, 11, 12, Rotation2d.fromRotations(0.110840), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), frontLeftPos); //TODO: update  the module offsets 
                FR = new SwerveModuleConstants(20, 21, 22, Rotation2d.fromRotations(0.265869), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), frontRightPos); //TODO: update  the module offsets
                BL = new SwerveModuleConstants(30, 31, 32, Rotation2d.fromRotations(-0.269043), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), backLeftPos); //TODO: update  the module offsets
                BR = new SwerveModuleConstants(40, 41, 42, Rotation2d.fromRotations(-0.158936), new Slot0Configs().withKS(driveKS).withKV(driveKV).withKA(driveKA).withKP(driveKP).withKD(driveKD).withKD(driveKD), backRightPos); //TODO: update  the module offsets
            }
        };
    }
}
