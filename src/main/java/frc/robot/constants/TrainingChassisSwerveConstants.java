package frc.robot.constants;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.DELib25.Subsystems.Drive.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Subsystems.Drive.SwerveUtil.SwerveConstants;


public final class TrainingChassisSwerveConstants implements SwerveConstants {

    public static final String MAC_ADDRESS = "00-80-2F-39-64-5B";

    private static final String CANBUS_NAME = "rio";
    private static final COTSTalonFXSwerveConstants CHOSEN_MODULE_CONSTANTS = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
    // Ports and IDs
    private static final int GYRO_ID = 0;

    // Swerve Modules
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 20;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 21 ;
    private static final int FRONT_LEFT_STEER_ENCODER_ID = 22;


    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 10;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 11;
    private static final int FRONT_RIGHT_STEER_ENCODER_ID = 12;

    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 30;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 31;
    private static final int BACK_LEFT_STEER_ENCODER_ID = 32;

    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 40;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 41;
    private static final int BACK_RIGHT_STEER_ENCODER_ID = 42;

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = CHOSEN_MODULE_CONSTANTS.wheelDiameter / 2.0;
    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = 0;//TODO: measure

    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = 0.51665;

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = 0.51665;
    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = CHOSEN_MODULE_CONSTANTS.getTheoreticalMaxLinearSpeedMps() * 0.9;//we multiply by 0.9 to be safe//the exstra .0 is to make it a double not a int so we dont have 0

    // CANcoder offsets of the swerve modules - bevel gears pointing left of the robot
    // private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = 0.381592;//TODO: remeasure
    // private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = 0.260498;//TODO: remeasure
    // private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = -0.135254;//TODO: remeasure
    // private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = -0.475342;//TODO: remeasure

    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = 0.60;
    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = 0.25;
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = 0.1;
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = 0;

    private static final int GYRO_MOUNTING_ANGLE = 0;

    //SysId configs
    public static final SysIdRoutine.Config TRANSLATION_SYS_ID_CONFIG = new SysIdRoutine.Config(
        null, Units.Volts.of(7), Units.Seconds.of(7),state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    );
    public static final SysIdRoutine.Config ROTATION_SYS_ID_CONFIG = new SysIdRoutine.Config(
        Units.Volts.of(Math.PI / 6).per(Units.Second), Units.Volts.of(Math.PI), Units.Seconds.of(5),state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    );
    public static final SysIdRoutine.Config STEER_SYS_ID_CONFIG = new SysIdRoutine.Config(
        null, Units.Volts.of(7), null,state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    );

    private static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] swerveModuleConstants = createSwerveModuleConstants();

    private static final SwerveDrivetrainConstants swerveDrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(CANBUS_NAME)
        .withPigeon2Id(GYRO_ID)
        .withPigeon2Configs(
            new Pigeon2Configuration()
                .withMountPose(new MountPoseConfigs().withMountPoseYaw(GYRO_MOUNTING_ANGLE))
        );
    

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] createSwerveModuleConstants(){
        @SuppressWarnings("unchecked")
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants = new SwerveModuleConstants[4];

        moduleConstants[0] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR_ID)
            .withSteerMotorId(FRONT_LEFT_STEER_MOTOR_ID)
            .withEncoderId(FRONT_LEFT_STEER_ENCODER_ID)
            .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2);
            //.withDriveMotorInverted(false);

        moduleConstants[1] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR_ID)
            .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR_ID)
            .withEncoderId(FRONT_RIGHT_STEER_ENCODER_ID)
            .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2);
            //.withDriveMotorInverted(true);

        moduleConstants[2] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR_ID)
            .withSteerMotorId(BACK_LEFT_STEER_MOTOR_ID)
            .withEncoderId(BACK_LEFT_STEER_ENCODER_ID)
            .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2);
            //.withDriveMotorInverted(false);

        moduleConstants[3] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR_ID)
            .withSteerMotorId(BACK_RIGHT_STEER_MOTOR_ID)
            .withEncoderId(BACK_RIGHT_STEER_ENCODER_ID)
            .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2);
            //.withDriveMotorInverted(true);

        return moduleConstants;
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getDefaultSwerveModuleConstants() {
        return new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorInitialConfigs(getDefaultDriveTalonFXConfig())
            .withSteerMotorInitialConfigs(getDefaultSteerTalonFXConfig())
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)//TODO: tune
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)//TODO: tune
            .withDriveMotorGains(new Slot0Configs().withKP(10).withKD(0))//TODO: tune
            .withSteerMotorGains(new Slot0Configs()
                .withKP(CHOSEN_MODULE_CONSTANTS.steerKP)
                .withKI(CHOSEN_MODULE_CONSTANTS.steerKI)
                .withKD(CHOSEN_MODULE_CONSTANTS.steerKD)
            )
            .withDriveMotorGearRatio(CHOSEN_MODULE_CONSTANTS.driveGearRatio)
            .withSteerMotorGearRatio(CHOSEN_MODULE_CONSTANTS.steerGearRatio)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(CHOSEN_MODULE_CONSTANTS.isDriveInverted())
            .withSteerMotorInverted(CHOSEN_MODULE_CONSTANTS.isSteerInverted())
            .withEncoderInverted(CHOSEN_MODULE_CONSTANTS.isCancoderInverted())
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            //.withSlipCurrent(120) //TODO: tune
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)//the FusedCANcoder is pro only
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);
    }

    private static TalonFXConfiguration getDefaultDriveTalonFXConfig() {
        return new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)//TODO: tune
                .withStatorCurrentLimitEnable(true)//TODO: tune
                .withSupplyCurrentLimit(25.0)//TODO: tune
                .withStatorCurrentLimit(60.0)//TODO: tune
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(CHOSEN_MODULE_CONSTANTS.driveMotorInvert)
                .withNeutralMode(NeutralModeValue.Brake)
        );
    }

    private static TalonFXConfiguration getDefaultSteerTalonFXConfig() {
        return new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)//TODO: tune
                    .withStatorCurrentLimitEnable(true)//TODO: tune
                    .withSupplyCurrentLimit(20.0)//TODO: tune
                    .withStatorCurrentLimit(40.0)//TODO: tune
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(CHOSEN_MODULE_CONSTANTS.steerMotorInvert)
                    .withNeutralMode(NeutralModeValue.Coast)
                    
            );
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getSwerveModuleConstants() {
        return swerveModuleConstants;
    }

    @Override public SwerveDrivetrainConstants getSwerveDrivetrainConstants() { return swerveDrivetrainConstants; }
    @Override public SysIdRoutine.Config getTranslationSysIdConfig() { return TRANSLATION_SYS_ID_CONFIG; }
    @Override public SysIdRoutine.Config getRotationSysIdConfig() { return ROTATION_SYS_ID_CONFIG; }
    @Override public SysIdRoutine.Config getSteerSysIdConfig() { return STEER_SYS_ID_CONFIG; }
    
}
