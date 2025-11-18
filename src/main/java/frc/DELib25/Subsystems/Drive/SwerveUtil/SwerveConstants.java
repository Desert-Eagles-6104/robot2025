package frc.DELib25.Subsystems.Drive.SwerveUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Holds all the constants for the robot swerve drivetrain and modules.
 * The drivetrain also includes the pigeon and gyro configurations.
 */
public abstract interface SwerveConstants {
    
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getSwerveModuleConstants();

    SwerveDrivetrainConstants getSwerveDrivetrainConstants();

    SysIdRoutine.Config getTranslationSysIdConfig();

    SysIdRoutine.Config getRotationSysIdConfig();

    SysIdRoutine.Config getSteerSysIdConfig();

}