package frc.DELib25.Subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveModuleConstants;

public class SwerveConstants {
    public COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
    COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

    /*String bus */
    String canBus = "Canivore";

    /* Drivetrain Constants */
    public double wheelCircumference = chosenModule.wheelCircumference;

    /*swerve module position*/
    public Translation2d frontLeftPos = new Translation2d(0.375,0.375);
    public Translation2d frontRightPos = new Translation2d(0.375,-0.375);
    public Translation2d backLeftPos = new Translation2d(-0.375,0.375);
    public Translation2d backRightPos = new Translation2d(-0.375,-0.375);
    public Translation2d[] modulesPositions = new Translation2d[4];

    /* Module Gear Ratios */
    public double driveGearRatio = chosenModule.driveGearRatio;
    public double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public SensorDirectionValue canCoderInvert = chosenModule.cancoderInvert;

    /*Feedback Sensor Azimuth */
    public FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    /* Swerve Current Limiting */
    public int angleContinuousCurrentLimit = 25;
    public int anglePeakCurrentLimit = 40;
    public double anglePeakCurrentDuration = 0.1;
    public boolean angleEnableCurrentLimit = true;

    public int driveStatorCurrentLimit = 60; //TODO: cheak when training
    public boolean driveEnableStatorCurrentLimit = true;
    public int driveContinuousCurrentLimit = 40;
    public int drivePeakCurrentLimit = 60;
    public double drivePeakCurrentDuration = 0.1;
    public boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
    * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public double openLoopRamp = 0.2;
    public double closedLoopRamp = 0.2;

    /* Angle Motor PID Values */
    public double angleKP = chosenModule.angleKP;
    public double angleKI = chosenModule.angleKI;
    public double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public double driveKP = 3.0; //TODO: This must be tuned to specific robot
    public double driveKI = 0.0;
    public double driveKD = 0.0;
    public double driveKF = 0.0;

    /* Heading PID Values */
    public double HeadingKP = 4;
    public double HeadingKI = 0.0;
    public double HeadingKD = 0;
    public double HeadingTolerence = 0;


    /* Drive Motor Characterization Values 
    * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public double driveKS = (0.0); //TODO: This must be tuned to specific robot
    public double driveKV = (0.0);
    public double driveKA = (0.0);

    /*wheel parameters */
    public static double WheelRadius = 0.0508;
    public static double WheelCircumference = WheelRadius * 2 * Math.PI;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public double maxSpeed = 5.2; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public NeutralMode angleNeutralMode = NeutralMode.Brake;
    public NeutralMode driveNeutralMode = NeutralMode.Brake;


    public SwerveModuleConstants FL = new SwerveModuleConstants(0, 0, 0, null, null, null);
    public SwerveModuleConstants FR = new SwerveModuleConstants(0, 0, 0, null, null, null);
    public SwerveModuleConstants BL = new SwerveModuleConstants(0, 0, 0, null, null, null);
    public SwerveModuleConstants BR = new SwerveModuleConstants(0, 0, 0, null, null, null);

    public String filepath = "/home/lvuser/natinst/ModuleOffsets.csv";

    private TalonFXConfiguration driveTalonFXConfigs;
    private TalonFXConfiguration steerTalonFXConfigs;

    public TalonFXConfiguration driveTalonFXConfigs(){
        TalonFXConfiguration configs = new TalonFXConfiguration();
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        configs.MotorOutput.Inverted = chosenModule.driveMotorInvert;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Gear Ratio Config */
        configs.Feedback.SensorToMechanismRatio = chosenModule.driveGearRatio;

        /* Current Limiting */
        configs.CurrentLimits.StatorCurrentLimit = driveStatorCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimit = driveContinuousCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLowerLimit = drivePeakCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLowerTime = drivePeakCurrentDuration;

        /* PID Config */
        configs.Slot0.kP = driveKP;
        configs.Slot0.kI = driveKI;
        configs.Slot0.kD = driveKD;

        /* Open and Closed Loop Ramping */
        configs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
        configs.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

        configs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
        configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;
        return configs;
    }

    public TalonFXConfiguration steerTalonFXConfigs(){
        TalonFXConfiguration configs = new TalonFXConfiguration();
         /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        configs.MotorOutput.Inverted = chosenModule.angleMotorInvert;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;  

        /* Gear Ratio and Wrapping Config */
        configs.Feedback.SensorToMechanismRatio = chosenModule.angleGearRatio;
        configs.Feedback.FeedbackSensorSource = feedbackSensorSource;
        configs.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        configs.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimit = angleContinuousCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimit = anglePeakCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLowerTime = anglePeakCurrentDuration;

        /* PID Config */
        configs.Slot0.kP = chosenModule.angleKP;
        configs.Slot0.kI = chosenModule.angleKI;
        configs.Slot0.kD = chosenModule.angleKD;
        return configs;
    }

    public CANcoderConfiguration canCoderConfigs(){
        CANcoderConfiguration configs = new CANcoderConfiguration();
        /** Swerve CANCoder Configuration */
        configs.MagnetSensor.SensorDirection = chosenModule.cancoderInvert;
        return configs;
    }
}
