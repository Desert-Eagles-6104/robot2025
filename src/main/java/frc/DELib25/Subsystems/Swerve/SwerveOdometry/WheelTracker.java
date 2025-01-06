package frc.DELib25.Subsystems.Swerve.SwerveOdometry;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Sensors.Pigeon;
import frc.DELib25.Subsystems.Swerve.SwerveModule;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class WheelTracker {
	private final Pigeon mPigeon = Pigeon.getInstance();
	private final SwerveModule[] mModules;

	private WheelProperties[] wheels = new WheelProperties[4];
	private Pose2d robotPose = new Pose2d();
	private Translation2d robotVelocity = new Translation2d();

	private double robotHeading;

	private double mTimestamp;
	@SuppressWarnings("unused")
	private boolean mIsEnabled = false;

	private BaseStatusSignal[] mAllSignals;

	private OdometryThread mOdometryThread;

	public WheelTracker(SwerveModule[] modules) {
		if (modules.length != 4) {
			throw new IllegalArgumentException("Odometry needs 4 modules to run");
		}

		mModules = modules;

		for (int i = 0; i < wheels.length; i++) {
			WheelProperties w = new WheelProperties();
			Translation2d robotToWheel = new Translation2d(
					Constants.Swerve.swerveConstants.modulesPositions[i].getX(),
					Constants.Swerve.swerveConstants.modulesPositions[i].getY());
			w.startingPosition = robotToWheel;
			wheels[i] = w;
		}

		resetModulePoses();

		mAllSignals = new BaseStatusSignal[(4 * 4) + 2];
		for (int i = 0; i < 4; ++i) {
			var signals = mModules[i].getOdometrySignals();
			mAllSignals[(i * 4) + 0] = signals[0];
			mAllSignals[(i * 4) + 1] = signals[1];
			mAllSignals[(i * 4) + 2] = signals[2];
			mAllSignals[(i * 4) + 3] = signals[3];
		}
		mAllSignals[mAllSignals.length - 2] = mPigeon.getYawStatusSignal();
		mAllSignals[mAllSignals.length - 1] = mPigeon.getRateStatusSignal();

		for (BaseStatusSignal sig : mAllSignals) {
			sig.setUpdateFrequency(250);
		}
		mOdometryThread = new OdometryThread();
		mOdometryThread.setDaemon(true);
		mOdometryThread.start();
	}

	public void start() {
		mIsEnabled = true;
	}

	public void stop() {
		mIsEnabled = false;
	}

	private class OdometryThread extends Thread {
		@Override
		public void run() {
			while (true) {
				try {
					robotHeading = mPigeon.getYaw().getDegrees();
					updateRobotPose(Timer.getFPGATimestamp());
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}

	private void updateRobotPose(double timestamp) {
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = Rotation2d.fromDegrees(robotHeading);

		double avg_delta = 0.0;
		double[] deltas = new double[4];
		for (int i = 0; i < mModules.length; i++) {
			SwerveModule m = mModules[i];
			WheelProperties w = wheels[i];
			updateWheelOdometry(m, w);
			double delta = w.estimatedRobotPose
					.getTranslation()
					.plus(robotPose.getTranslation().unaryMinus())
					.getNorm();
			deltas[i] = delta;
			avg_delta += delta;
		}
		avg_delta /= 4;

		int min__dev_idx = 0;
		double min_dev = Double.MAX_VALUE;
		List<WheelProperties> accurateModules = new ArrayList<>();
		for (int i = 0; i < deltas.length; i++) {
			WheelProperties w = wheels[i];
			double dev = Math.abs(deltas[i] - avg_delta);
			if (dev < min_dev) {
				min_dev = dev;
				min__dev_idx = i;
			}
			if (dev <= 0.01 && w.useEstimatedRobotPose) {
				accurateModules.add(w);
			}
		}

		if (accurateModules.isEmpty()) {
			accurateModules.add(wheels[min__dev_idx]);
		}

		int n = accurateModules.size();

		SmartDashboard.putNumber("Modules Used For Odometry", n);

		for (WheelProperties w : accurateModules) {
			x += w.estimatedRobotPose.getTranslation().getX();
			y += w.estimatedRobotPose.getTranslation().getY();
		}
		final Pose2d new_pose = new Pose2d(new Translation2d(x / n, y / n), heading);

		robotPose = new_pose;
		resetModulePoses(robotPose);
	}

	private void updateWheelOdometry(SwerveModule module, WheelProperties props) {
		double currentEncDistance = module.getModulePosition().distanceMeters;
		double deltaEncDistance = currentEncDistance - props.previousEncDistance;
		Rotation2d wheelAngle = module.getAngle().rotateBy(Rotation2d.fromDegrees(robotHeading));
		Translation2d deltaPosition = new Translation2d(wheelAngle.getCos() * deltaEncDistance, wheelAngle.getSin() * deltaEncDistance);
		Translation2d updatedPosition = props.position.plus(deltaPosition);
		Pose2d wheelPose = new Pose2d(updatedPosition, Rotation2d.fromDegrees(robotHeading));
		props.estimatedRobotPose = wheelPose.transformBy(new Transform2d(props.startingPosition.unaryMinus(), Rotation2d.fromDegrees(0)));

		props.position = updatedPosition;
		props.previousEncDistance = currentEncDistance;
	}

	private void resetModulePoses(Pose2d robotPose) {
		for (int i = 0; i < mModules.length; i++) {
			WheelProperties props = wheels[i];
			Translation2d modulePosition = robotPose
					.transformBy(new Transform2d(props.startingPosition, Rotation2d.fromDegrees(0)))
					.getTranslation();
			props.position = modulePosition;
		}
	}

	private void resetModulePoses() {
		for (int i = 0; i < mModules.length; i++) {
			WheelProperties props = wheels[i];
			props.position = props.startingPosition;
		}
	}

	public void resetPose(Pose2d pose) {
		robotPose = pose;
		resetModulePoses(robotPose);
	}

	public class WheelProperties {
		private double previousEncDistance = 0;
		private Translation2d position;
		private Translation2d startingPosition;
		private Pose2d estimatedRobotPose = new Pose2d();
		private boolean useEstimatedRobotPose = true;
	}

	public void ignoreModule(boolean FL, boolean FR, boolean BL, boolean BR){
		wheels[0].useEstimatedRobotPose = FL;
		wheels[1].useEstimatedRobotPose = FR;
		wheels[2].useEstimatedRobotPose = BL;
		wheels[3].useEstimatedRobotPose = BR;
	}

	public BaseStatusSignal[] getAllodometrySignals(){
		return mAllSignals;
	}

	public synchronized Pose2d getRobotPose() {
		return robotPose;
	}

	public Translation2d getMeasuredVelocity() {
		return robotVelocity;
	}

	public double getTimestamp() {
		return mTimestamp;
	}

	public double wheel0_x() {
		return wheels[0].position.getX();
	}

	public double wheel0_y() {
		return wheels[0].position.getY();
	}

	public double wheel1_x() {
		return wheels[1].position.getX();
	}

	public double wheel1_y() {
		return wheels[1].position.getY();
	}

	public double wheel2_x() {
		return wheels[2].position.getX();
	}

	public double wheel2_y() {
		return wheels[2].position.getY();
	}

	public double wheel3_x() {
		return wheels[3].position.getX();
	}

	public double wheel3_y() {
		return wheels[3].position.getY();
	}

	public double robot_x() {
		return robotPose.getTranslation().getX();
	}

	public double robot_y() {
		return robotPose.getTranslation().getY();
	}
}
