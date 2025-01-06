package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int steeringMotorID;
    public final int absoluteEncoderID;
    public final Rotation2d angleOffset;
    public final Slot0Configs slot0Configs;
    public final Translation2d modulePosition;

    public SwerveModuleConstants (int driveMotorID, int steeringMotorID, int absoluteEncoderID , Rotation2d angleOffset, Slot0Configs slot0Configs, Translation2d modulePosition){
        this.driveMotorID = driveMotorID;
        this.steeringMotorID = steeringMotorID;
        this.absoluteEncoderID = absoluteEncoderID;
        this.angleOffset = angleOffset;
        this.slot0Configs = slot0Configs;
        this.modulePosition = modulePosition;
    }
}