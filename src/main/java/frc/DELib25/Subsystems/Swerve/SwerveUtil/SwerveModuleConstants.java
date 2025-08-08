package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int steeringMotorID;
    public final int absoluteEncoderID;
    public final Rotation2d angleOffset;
    public final Translation2d modulePosition;

    public SwerveModuleConstants (int driveMotorID, int steeringMotorID, int absoluteEncoderID , Rotation2d angleOffset, Translation2d modulePosition){
        this.driveMotorID = driveMotorID;
        this.steeringMotorID = steeringMotorID;
        this.absoluteEncoderID = absoluteEncoderID;
        this.angleOffset = angleOffset;
        this.modulePosition = modulePosition;
    }
}