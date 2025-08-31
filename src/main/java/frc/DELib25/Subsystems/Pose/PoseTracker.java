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
    private static PoseTracker instance;

    public static PoseTracker getInstance() {
        if (instance == null) {
            instance = new PoseTracker();
        }
        return instance;
    }

    private Pose2d pose;
    private LimelightHelpers.PoseEstimate limelightMeserment;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> pastPoses;
    private Field2d field;

    public PoseTracker() {
        this.pose = new Pose2d();
        this.pastPoses = new InterpolatingTreeMap<>(51);
        this.field = new Field2d();
        SmartDashboard.putData("Field", this.field);
    }

    public void updatePose(Pose2d newPose, double timestamp) {
        this.pose = newPose;
        this.pastPoses.put(new InterpolatingDouble(timestamp), newPose);

        SmartDashboard.putNumber("RobotHeading", this.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("robotX", this.pose.getX());
        SmartDashboard.putNumber("robotY ", this.pose.getY());
        SmartDashboard.putNumber("robotorientation", this.pose.getRotation().getDegrees());

        this.field.setRobotPose(this.pose);
    }

    public void updateLimelightMeasurement(LimelightHelpers.PoseEstimate newMeasurement) {
        this.limelightMeserment = newMeasurement;
    }

    public Pose2d getPose() {
        return this.pose;
    }

    public Rotation2d getHeading() {
        return this.pose.getRotation();
    }

    public Pose2d getPoseLatencyAgo(double latencySeconds) {
        double timestamp = Timer.getFPGATimestamp() - latencySeconds;
        return getPoseTimestamp(timestamp);
    }

    public Pose2d getPoseTimestamp(double timestamp) {
        return this.pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public LimelightHelpers.PoseEstimate getLimelightMeasurement() {
        return this.limelightMeserment;
    }
}
