package frc.robot.subsystems.RobotSwerveUtil;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Util.FieldUtil;
import frc.DELib25.Subsystems.Drive.SwerveSubsystem;
import frc.DELib25.Subsystems.Pose.PoseTracker;
import frc.robot.subsystems.VisionSubsystemRobot2025;

public class ReefAssist extends Command {
	private final SwerveSubsystem swerve;
	private final VisionSubsystemRobot2025 vision;

	private final BooleanSupplier chooseRight;
	private final Supplier<Translation2d> leftErrorSupplier;
	private final Supplier<Translation2d> rightErrorSupplier;

	private final double kpSide = 2.0;
	private final double kpForward = 2.7;

	private final LinearFilter filterSide = LinearFilter.movingAverage(4);
	private final LinearFilter filterForward = LinearFilter.movingAverage(4);

	private final StableBoolean lostTagFor = new StableBoolean(0.5);

	private final SwerveRequest.ApplyFieldSpeeds request = new SwerveRequest.ApplyFieldSpeeds()
			.withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

	public ReefAssist(SwerveSubsystem swerve, VisionSubsystemRobot2025 vision, BooleanSupplier chooseRight, Supplier<Translation2d> leftErrorSupplier, Supplier<Translation2d> rightErrorSupplier) {
		this.swerve = swerve;
		this.vision = vision;
		this.chooseRight = chooseRight;
		this.leftErrorSupplier = leftErrorSupplier;
		this.rightErrorSupplier = rightErrorSupplier;
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		if (lostTagFor.update(!vision.getTv())) {
			swerve.disableMotors();
			return;
		}

		Translation2d err = chooseRight.getAsBoolean()
				? rightErrorSupplier.get()
				: leftErrorSupplier.get();

		if (err == null) {
			return;
		}

		double fwd = filterForward.calculate(-err.getX()) * kpForward;
		double str = filterSide.calculate(-err.getY()) * kpSide;

		if (!FieldUtil.isBlueAllianceOrDefault()) {
			fwd = -fwd;
			str = -str;
		}

		// Convert to FIELD-relative using current heading
		ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(fwd, str, 0.0, PoseTracker.getInstance().getHeading());
		swerve.getIO().setSwerveState(request.withSpeeds(fieldSpeeds));
	}

	@Override
	public void end(boolean interrupted) {
		swerve.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
