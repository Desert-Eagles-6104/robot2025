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
    private COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
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
    private InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    private InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public SensorDirectionValue canCoderInvert = chosenModule.cancoderInvert;

    /*Feedback Sensor Azimuth */
    public FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    /* Swerve Current Limiting */
    private int angleContinuousCurrentLimit = 25;
    private int anglePeakCurrentLimit = 40;
    private double anglePeakCurrentDuration = 0.1;
    private boolean angleEnableCurrentLimit = true;

    private int driveStatorCurrentLimit = 60; //TODO: cheak when training
    private boolean driveEnableStatorCurrentLimit = true;
    private int driveContinuousCurrentLimit = 40;
    private int drivePeakCurrentLimit = 60;
    private double drivePeakCurrentDuration = 0.1;
    private boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
    * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    private double openLoopRamp = 0.2;
    private double closedLoopRamp = 0.2;

    /* Angle Motor PID Values */
    private double angleKP = chosenModule.angleKP;
    private double angleKI = chosenModule.angleKI;
    private double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    private double driveKP = 3.0; //TODO: This must be tuned to specific robot
    private double driveKI = 0.0;
    private double driveKD = 0.0;
    private double driveKF = 0.0;

    /* Heading PID Values */
    public double HeadingKP = 4;
    public double HeadingKI = 0.0;
    public double HeadingKD = 0;
    public double HeadingTolerence = 0;


    /* Drive Motor Characterization Values 
    * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public double driveKS = (0.0); //TODO: This must be tuned to specific robot
    private double driveKV = (0.0);
    private double driveKA = (0.0);

    /*wheel parameters */
    private static double WheelRadius = 0.0508;
    private static double WheelCircumference = WheelRadius * 2 * Math.PI;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public double maxSpeed = 5.2; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    private NeutralMode angleNeutralMode = NeutralMode.Brake;
    private NeutralMode driveNeutralMode = NeutralMode.Brake;


    public SwerveModuleConstants FL;
    public SwerveModuleConstants FR;
    public SwerveModuleConstants BL;
    public SwerveModuleConstants BR;

    public String filepath = "/home/lvuser/natinst/ModuleOffsets.csv";

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
