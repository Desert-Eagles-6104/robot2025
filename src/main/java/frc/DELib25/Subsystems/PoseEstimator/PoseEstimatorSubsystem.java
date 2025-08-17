package frc.DELib25.Subsystems.PoseEstimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Intepulation.InterpolatingDouble;
import frc.DELib25.Intepulation.InterpolatingTreeMap;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;
import frc.DELib25.Subsystems.drive.SwerveSubsystem;

/** Creates a new PoseEstimator. */
public class PoseEstimatorSubsystem extends SubsystemBase {

  private SwerveSubsystem swerve;
  private LimelightHelpers.PoseEstimate limelightMesermentMT2;
  private boolean first;
  private StableBoolean tvStableBoolean;
  private SwerveDrivePoseEstimator odometry;
  private Field2d field;
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> pastPoses;
  private VisionSubsystem vision;

  public PoseEstimatorSubsystem(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;

    this.odometry = new SwerveDrivePoseEstimator(
        this.swerve.getIO().getKinematics(),
        Rotation2d.fromDegrees(0),
        this.swerve.getIO().getModulesPositions(), new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.3, 0.3, 9999999));

    this.first = true;
    this.tvStableBoolean = new StableBoolean(0.15);
    this.field = new Field2d();

    SmartDashboard.putData("Field", this.field);

    this.pastPoses = new InterpolatingTreeMap<>(51); // Represents the max pose
                                                     // history size
  }

  @Override
  public void periodic() {

    if (!this.first) {
      updateVisionOdometry();
    } else {
      this.first = false;
    }

    Pose2d currentPose = this.updateOdometry();
    this.pastPoses.put(new InterpolatingDouble(Timer.getFPGATimestamp()), currentPose);
    SmartDashboard.putNumber("RobotHeading", getHeading().getDegrees());
    SmartDashboard.putNumber("robotX", this.getPose().getX());
    SmartDashboard.putNumber("robotY ", this.getPose().getY());
    SmartDashboard.putNumber("robotorientation", this.getPose().getRotation().getDegrees());

    this.updateOdometry();
    this.field.setRobotPose(this.odometry.getEstimatedPosition());
  }

  public void updateVisionOdometry() {
    if (!this.first) {
      boolean rejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight-april", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      this.limelightMesermentMT2 = this.vision.getEstimatedRobotPose();
      if (Math.abs(this.swerve.getIO().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 360 || this.limelightMesermentMT2.pose == null) {
        rejectUpdate = true;
      }
      if (!rejectUpdate && this.tvStableBoolean.update(this.vision.getTv()) && this.limelightMesermentMT2.pose != null) {
        this.addVisionMeasurement(this.limelightMesermentMT2.pose, this.limelightMesermentMT2.timestampSeconds);
      }
    } else {
      this.first = false;
    }
  }

  public void resetOdometryToPose(Pose2d pose) {
    this.odometry.resetPosition(this.swerve.getIO().getYaw(), this.swerve.getIO().getModulesPositions(), pose);
  }

  public void zeroHeading() {
    Rotation2d heading;
    if (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
      heading = Rotation2d.fromDegrees(180);
    } else {
      heading = new Rotation2d();
    }
    this.odometry.resetRotation(heading);
  }

  public Pose2d updateOdometry() {
    return this.odometry.update(this.swerve.getIO().getYaw(), this.swerve.getIO().getModulesPositions());
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    this.odometry.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(0.7, 0.7, 9999999));
  }

  public Pose2d getPose() {
    return this.odometry.getEstimatedPosition();
  }

  public Rotation2d getHeading() {
    return this.getPose().getRotation();
  }

  public void resetPositionFromCamera() {
    if (this.limelightMesermentMT2.pose != null) {
      this.resetOdometryToPose(this.limelightMesermentMT2.pose);
    }
  }

  public Pose2d getInterpolatedPose(double latencySeconds) {
    double timestamp = Timer.getFPGATimestamp() - latencySeconds;
    return this.pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
  }
}
