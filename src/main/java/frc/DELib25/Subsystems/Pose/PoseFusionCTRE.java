package frc.DELib25.Subsystems.Pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.Drive.SwerveIOCTRE;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraType;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class PoseFusionCTRE extends SubsystemBase {

	private static final double STARTUP_DELAY_SEC = 0.20;
	private static final double MAX_GYRO_RATE_DPS = 360.0;
	private static final double MAX_VISION_POS_ERR_M = 1.0;

	private final SwerveIOCTRE swerveIO;
	private final VisionSubsystem vision;

	private final StableBoolean tvStable;
	private final double enableVisionAt;

	public PoseFusionCTRE(SwerveIOCTRE swerveIO, VisionSubsystem vision) {
		this.swerveIO = swerveIO;
		this.vision = vision;
		this.tvStable = new StableBoolean(0.15);
		this.enableVisionAt = Timer.getFPGATimestamp() + STARTUP_DELAY_SEC;
	}

  	public void fuse() {
		if (Timer.getFPGATimestamp() >= this.enableVisionAt) {
			this.maybeFuseVision();
		}
		PoseTracker.updatePose(this.getPose());
  	}

	private void maybeFuseVision() {
		Pose2d now = this.swerveIO.getState().Pose;

		LimelightHelpers.SetRobotOrientation(
			CameraType.AprilTagCamera.getCameraName(),
			now.getRotation().getDegrees(), 
			0, 0, 0, 0, 0
		);

		LimelightHelpers.PoseEstimate est = this.vision.getEstimatedRobotPose();
		if (!this.isVisionUsable(est, now)) return;

		PoseTracker.updateLimelightMeasurement(est);
		this.swerveIO.addVisionMeasurement(est.pose, est.timestampSeconds);
	}

	private boolean isVisionUsable(LimelightHelpers.PoseEstimate est, Pose2d now) {
		if (est == null || est.pose == null) return false;

		if (!this.tvStable.update(this.vision.getTv())) return false;

		double rateDps = Math.abs(this.swerveIO.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
		if (rateDps > MAX_GYRO_RATE_DPS) return false;

		double dx = est.pose.getX() - now.getX();
		double dy = est.pose.getY() - now.getY();
		if (Math.hypot(dx, dy) > MAX_VISION_POS_ERR_M) return false;

		return true;
	}
	
	public Pose2d getPoseLatencyAgo(double latencySeconds) {
		double timestamp = Timer.getFPGATimestamp() - latencySeconds;
		return this.swerveIO.samplePoseAt(timestamp).orElse(null);
	}

	public Pose2d getPose() { return this.swerveIO.getPose(); }
	public Rotation2d getHeading() { return this.getPose().getRotation(); }
}