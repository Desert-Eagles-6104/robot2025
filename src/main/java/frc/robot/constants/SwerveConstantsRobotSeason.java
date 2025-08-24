package frc.robot.constants;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.DELib25.Subsystems.Drive.SwerveUtil.COTSTalonFXSwerveConstants;

/**
 * Holds all the constants for the robot swerve drivetrain and modules.
 * The drivetrain also includes the pigeon and gyro configurations.
 */
public final class SwerveConstantsRobotSeason {
    
    private static final String CANBUS_NAME = "Canivore";
    private static final COTSTalonFXSwerveConstants CHOSEN_MODULE_CONSTANTS = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);
    // Ports and IDs
    private static final int GYRO_ID = 44;

    // Swerve Modules
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 10;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 11;
    private static final int FRONT_LEFT_STEER_ENCODER_ID = 12;


    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 20;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 21;
    private static final int FRONT_RIGHT_STEER_ENCODER_ID = 22;

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
     * Ratio between the drive motor shaft and the output shaft the wheel is mounted on.
     */
    private static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE_CONSTANTS.driveGearRatio;

    /**
     * Ratio between the steer motor shaft and the steer output shaft.
     */
    private static final double STEER_GEAR_RATIO = CHOSEN_MODULE_CONSTANTS.steerGearRatio;

    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = 150/7/1;//TODO: tune

    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = 0.61665;

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = 0.61665;

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = 5.2;//TODO: tune (i used the value from the pre code)

    /**
     * The maximum angular speed of the robot in radians per second.
     * If set to 0, the value is calculated using the max speed in meters per second
     * and the wheelbase radius.
     */

    // CANcoder offsets of the swerve modules - bevel gears pointing left of the robot
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = -0.396240;
    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = 0.279297;
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = 0.226562;
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = -0.157227;
    
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
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)//TODO: tune
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)//TODO: tune
            .withDriveMotorGains(new Slot0Configs().withKP(8))//TODO: tune
            .withSteerMotorGains(new Slot0Configs()
                .withKP(CHOSEN_MODULE_CONSTANTS.steerKP)
                .withKI(CHOSEN_MODULE_CONSTANTS.steerKI)
                .withKD(CHOSEN_MODULE_CONSTANTS.steerKD)
            )
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withDriveMotorInverted(CHOSEN_MODULE_CONSTANTS.driveMotorInvert == InvertedValue.CounterClockwise_Positive)
            .withSteerMotorInverted(CHOSEN_MODULE_CONSTANTS.steerMotorInvert == InvertedValue.CounterClockwise_Positive)
            .withEncoderInverted(CHOSEN_MODULE_CONSTANTS.cancoderInvert == SensorDirectionValue.Clockwise_Positive)
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

    public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getSwerveModuleConstants() {
        return swerveModuleConstants;
    }

    public static SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
        return swerveDrivetrainConstants;
    }

    
}
