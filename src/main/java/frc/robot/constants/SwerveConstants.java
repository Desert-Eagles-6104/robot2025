package frc.robot.constants;
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

import frc.DELib25.Subsystems.Drive.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.DELib25.Util.ProjectConstants;
/**
 * Holds all the constants for the robot swerve drivetrain and modules.
 * The drivetrain also includes the pigeon and gyro configurations.
 */
public final class SwerveConstants {
    
    private static final String CANIVORE_CANBUS_NAME = "CANivore";//TODO: tune
    private static final COTSTalonFXSwerveConstants CHOSEN_MODULE_CONSTANTS = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);
    // Ports and IDs
    private static final int GYRO_ID = ProjectConstants.ERROR_CODE;//TODO: tune

    // Swerve Modules
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int FRONT_LEFT_STEER_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int FRONT_LEFT_STEER_ENCODER_ID = ProjectConstants.ERROR_CODE;//TODO: tune


    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int FRONT_RIGHT_STEER_ENCODER_ID = ProjectConstants.ERROR_CODE;//TODO: tune

    private static final int BACK_LEFT_DRIVE_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int BACK_LEFT_STEER_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int BACK_LEFT_STEER_ENCODER_ID = ProjectConstants.ERROR_CODE;//TODO: tune

    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int BACK_RIGHT_STEER_MOTOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final int BACK_RIGHT_STEER_ENCODER_ID = ProjectConstants.ERROR_CODE;//TODO: tune

    
    // Switch
    public static final int NEUTRAL_MODE_SWITCH_ID = ProjectConstants.ERROR_CODE;//TODO: tune
    public static final int HOME_BUTTON_ID = ProjectConstants.ERROR_CODE;//TODO: tune

    // Color sensor
    public static final int CANANDCOLOR_ID = ProjectConstants.ERROR_CODE;//TODO: tune

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = CHOSEN_MODULE_CONSTANTS.wheelDiameter / 2.0;

    private static final double DRIVE_MOTOR_PINION_TEETH_COUNT = ProjectConstants.ERROR_CODE;//TODO: tune
    /**
     * Ratio between the drive motor shaft and the output shaft the wheel is mounted on.
     */
    private static final double DRIVE_GEAR_RATIO = ProjectConstants.ERROR_CODE;//TODO: tune

    /**
     * Ratio between the steer motor shaft and the steer output shaft.
     */
    private static final double STEER_GEAR_RATIO = ProjectConstants.ERROR_CODE;//TODO: tune

    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = ProjectConstants.ERROR_CODE / DRIVE_MOTOR_PINION_TEETH_COUNT;//TODO: tune

    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = ProjectConstants.ERROR_CODE;//TODO: tune

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = ProjectConstants.ERROR_CODE;//TODO: tune

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = ProjectConstants.ERROR_CODE;//TODO: tune

    /**
     * The maximum angular speed of the robot in radians per second.
     * If set to 0, the value is calculated using the max speed in meters per second
     * and the wheelbase radius.
     */

    // CANcoder offsets of the swerve modules - bevel gears pointing left of the robot
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = ProjectConstants.ERROR_CODE;//TODO: tune

    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = ProjectConstants.ERROR_CODE;//TODO: tune
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = ProjectConstants.ERROR_CODE;//TODO: tune

    private static final int GYRO_MOUNTING_ANGLE = ProjectConstants.ERROR_CODE;//TODO: tune


    private static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] swerveModuleConstants = createSwerveModuleConstants();

    private static final SwerveDrivetrainConstants swerveDrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(CANIVORE_CANBUS_NAME)
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

        moduleConstants[1] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR_ID)
            .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR_ID)
            .withEncoderId(FRONT_RIGHT_STEER_ENCODER_ID)
            .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2);

        moduleConstants[2] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR_ID)
            .withSteerMotorId(BACK_LEFT_STEER_MOTOR_ID)
            .withEncoderId(BACK_LEFT_STEER_ENCODER_ID)
            .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2);

        moduleConstants[3] = getDefaultSwerveModuleConstants()
            .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR_ID)
            .withSteerMotorId(BACK_RIGHT_STEER_MOTOR_ID)
            .withEncoderId(BACK_RIGHT_STEER_ENCODER_ID)
            .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
            .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
            .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2);
        return moduleConstants;
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getDefaultSwerveModuleConstants() {
        return new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorInitialConfigs(getDefaultDriveTalonFXConfig())
            .withSteerMotorInitialConfigs(getDefaultSteerTalonFXConfig())
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)//TODO: tune
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)//TODO: tune
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)//TODO: tune
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)//TODO: tune
            .withDriveMotorGains(new Slot0Configs().withKP(36))//TODO: tune
            .withSteerMotorGains(new Slot0Configs()
                .withKP(CHOSEN_MODULE_CONSTANTS.steerKD)
                .withKI(CHOSEN_MODULE_CONSTANTS.steerKI)
                .withKD(CHOSEN_MODULE_CONSTANTS.steerKD)
            )//TODO: tune
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(false)//TODO: tune
            .withSteerMotorInverted(false)//TODO: tune
            .withEncoderInverted(false)//TODO: tune
            .withEncoderInitialConfigs(new CANcoderConfiguration())
            .withDriveFrictionVoltage(0.25)//TODO: tune
            .withSteerFrictionVoltage(0.001)//TODO: tune
            .withDriveInertia(0.001)//TODO: tune
            .withSteerInertia(0.00001)//TODO: tune
            .withSlipCurrent(120) //TODO: tuneE
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
            .withWheelRadius(WHEEL_RADIUS_METERS);
    }

    private static TalonFXConfiguration getDefaultDriveTalonFXConfig() {
        return new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)//TODO: tune
                .withStatorCurrentLimitEnable(true)//TODO: tune
                .withSupplyCurrentLimit(50.0)//TODO: tune
                .withStatorCurrentLimit(100.0)//TODO: tune
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
                    .withSupplyCurrentLimit(30.0)//TODO: tune
                    .withStatorCurrentLimit(90.0)//TODO: tune
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(CHOSEN_MODULE_CONSTANTS.steerMotorInvert)
                    .withNeutralMode(NeutralModeValue.Coast)
            );
    }

    public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getSwerveModuleConstants() {
        return swerveModuleConstants;
    }

    public static SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
        return swerveDrivetrainConstants;
    }
}
