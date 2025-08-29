package frc.DELib25.Subsystems.Pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Interpolation.InterpolatingDouble;
import frc.DELib25.Interpolation.InterpolatingTreeMap;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class PoseTracker {
    private static Pose2d pose;
    private static LimelightHelpers.PoseEstimate limelightMeserment;
    private static InterpolatingTreeMap<InterpolatingDouble, Pose2d> pastPoses;
    private static Field2d field;

    static {
        pose = new Pose2d();
        pastPoses = new InterpolatingTreeMap<>(51);
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public static void updatePose(Pose2d newPose) {
        pose = newPose;
        pastPoses.put(new InterpolatingDouble(Timer.getFPGATimestamp()), newPose);

        SmartDashboard.putNumber("RobotHeading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("robotX", pose.getX());
        SmartDashboard.putNumber("robotY ", pose.getY());
        SmartDashboard.putNumber("robotorientation", pose.getRotation().getDegrees());

        field.setRobotPose(pose);
    }

    public static void updateLimelightMeasurement(LimelightHelpers.PoseEstimate newMeasurement) {
        limelightMeserment = newMeasurement;
    }

    public static Pose2d getPose() {
        return pose;
    }
    
    public static Rotation2d getHeading() {
        return pose.getRotation();
    }

    public Pose2d getPoseLatencyAgo(double latencySeconds) {
		double timestamp = Timer.getFPGATimestamp() - latencySeconds;
		return pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
	}

    public static LimelightHelpers.PoseEstimate getLimelightMeasurement() {
        return limelightMeserment;
    }
}
