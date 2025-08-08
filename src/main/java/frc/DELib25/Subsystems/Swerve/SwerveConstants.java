package frc.DELib25.Subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib25.Motors.TalonFXConfigsCloner;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveModuleConstants;

public class SwerveConstants {
    private COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
    COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

    /*String bus */
    public String canBus = "Canivore";

    /* Drivetrain Constants */
    public double wheelCircumference = chosenModule.wheelCircumference;

    /* Angle Encoder Invert */
    public SensorDirectionValue canCoderInvert = chosenModule.cancoderInvert;

    /*Feedback Sensor Azimuth */
    public FeedbackSensorSourceValue feedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    /* Heading PID Values */
    public double HeadingKP = 4;
    public double HeadingKI = 0.0;
    public double HeadingKD = 0;
    public double HeadingTolerence = 0;


    /*wheel parameters */
    public static double WheelRadius = 0.0508;
    public static double WheelCircumference = WheelRadius * 2 * Math.PI;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public double maxSpeed = 5.2; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot


    public SwerveModuleConstants FL;
    public SwerveModuleConstants FR;
    public SwerveModuleConstants BL;
    public SwerveModuleConstants BR;

    public String filepath = "/home/lvuser/natinst/ModuleOffsets.csv";

    private TalonFXConfiguration driveTalonFXConfigs;
    private TalonFXConfiguration angleTalonFXConfigs;

    public SwerveConstants(TalonFXConfiguration driveTalonFXConfigs,TalonFXConfiguration angleTalonFXConfigs){
        this.driveTalonFXConfigs = driveTalonFXConfigs;
        this.angleTalonFXConfigs = angleTalonFXConfigs;
    }

    public TalonFXConfiguration getDriveTalonFXConfigs(){
        return TalonFXConfigsCloner.essentialOnlyClone(this.driveTalonFXConfigs);
    }

    public TalonFXConfiguration getAngleTalonFXConfiguration(){
        return TalonFXConfigsCloner.essentialOnlyClone(this.angleTalonFXConfigs);
    }

    public CANcoderConfiguration canCoderConfigs(){
        CANcoderConfiguration configs = new CANcoderConfiguration();
        /** Swerve CANCoder Configuration */
        configs.MagnetSensor.SensorDirection = chosenModule.cancoderInvert;
        return configs;
    }

    public Translation2d[] getModulesPositions() {
        Translation2d[] positions = new Translation2d[]{
            FL.modulePosition,
            FR.modulePosition,
            BL.modulePosition,
            BR.modulePosition
        };
        return positions;
    }
}
