package frc.DELib25.Subsystems.Drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.DELib25.Util.FieldUtil;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;


public class SwerveSubsystem extends SubsystemBase {
    private static final double CONTROLLER_DEADBAND = 0.1;
    public static final double TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS = Units.inchesToMeters(0.5);
    public static final double TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS_DRIVE_TO_POINT = Units.inchesToMeters(1.0);
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;

    public static final double ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK_DEGREES = 10.0;

    private static final double SKEW_COMPENSATION_SCALAR = -0.03;

    private static Pose2d robotToFieldFromSwerveDriveOdometry = new Pose2d();

    private double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

    private final PIDController choreoXController = new PIDController(7, 0, 0);
    private final PIDController choreoYController = new PIDController(7, 0, 0);
    private final PIDController choreoThetaController = new PIDController(7, 0, 0);

    private final PIDController autoDriveToPointController = new PIDController(3.0, 0, 0.1);
    private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    

    private DriveState systemState = DriveState.TELEOP_DRIVE;
    private DriveState wantedState = DriveState.TELEOP_DRIVE;

    private Rotation2d desiredRotationForRotationLockState;

    private Trajectory<SwerveSample> desiredChoreoTrajectory;
    private final Timer choreoTimer = new Timer();
    private Optional<SwerveSample> choreoSampleToBeApplied;

    private Pose2d desiredPoseForDriveToPoint = new Pose2d();

    private final SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();
    
    ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

    private final Object moduleIOLock = new Object();

    private SwerveIOCTRE io;
    private final CommandPS5Controller controller;

    private final double maxVelocity;
    private final double maxAngularVelocity;

    private double teleopVelocityCoefficient = 1.0;
    private double rotationVelocityCoefficient = 1.0;
    private double maximumAngularVelocityForDriveToPoint = Double.NaN;

    private final SysIdRoutine translationSysIdRoutine;
    /*
     * SysId routine for characterizing rotation. This is used to find PID gains
     * for the FieldCentricFacingAngle HeadingController.
     */
    private final SysIdRoutine rotationSysIdRoutine;

    private final SysIdRoutine steerSysIdRoutine;

    public SwerveSubsystem(SwerveIOCTRE io, CommandPS5Controller controller, double maxVelocity, double maxAngularVelocity,SysIdRoutine.Config translationSysIdRoutineConfig,
    SysIdRoutine.Config rotationSysIdRoutineConfig, SysIdRoutine.Config steerSysIdRoutineConfig) {
        this.io = io;
        this.controller = controller;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;

        // io.registerTelemetryFunction(swerveInputs);
        this.driveAtAngle.HeadingController = new PhoenixPIDController(5, 0, 0);

        this.driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        this.choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.translationSysIdRoutine = new SysIdRoutine(translationSysIdRoutineConfig, this.createSysIdRoutineMechanism());

        this.rotationSysIdRoutine = new SysIdRoutine(rotationSysIdRoutineConfig, this.createSysIdRoutineMechanism());

        this.steerSysIdRoutine = new SysIdRoutine(steerSysIdRoutineConfig, this.createSysIdRoutineMechanism());
    }

    private SysIdRoutine.Mechanism createSysIdRoutineMechanism() {
        return new SysIdRoutine.Mechanism(
            output -> this.io.setSwerveState(this.translationCharacterization.withVolts(output)),
            null,this
        );
    }

    private SysIdRoutine getRoutine(SysIdMechanism mechanism) {
        return switch (mechanism) {
        case SWERVE_TRANSLATION -> this.translationSysIdRoutine;
        case SWERVE_ROTATION -> this.rotationSysIdRoutine;
        case SWERVE_STEER -> this.steerSysIdRoutine;
        default -> throw new IllegalArgumentException(
                String.format("Mechanism %s is not supported.", mechanism));
        };
    }

    /**
     * Runs the quasi-static SysId test in the given direction for the given
     * mechanism.
     *
     * @param mechanism The mechanism to characterize
     * @param direction Direction of the quasi-static SysId test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
        return this.getRoutine(mechanism).quasistatic(direction);
    }

    /**
     * Runs the dynamic SysId test in the given direction for the given
     * mechanism.
     *
     * @param mechanism The mechanism to characterize
     * @param direction Direction of the dynamic SysId test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
        return this.getRoutine(mechanism).dynamic(direction);
    }

    @Override
    public void periodic() {
        this.io.updateSwerveInputs(this.swerveInputs);

        this.io.refreshData();
        this.io.updateModuleInputs(this.frontLeftInputs, this.frontRightInputs, this.backLeftInputs, this.backRightInputs);

        Logger.processInputs("Subsystems/Drive", this.swerveInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Front Left", this.frontLeftInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Front Right", this.frontRightInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Back Left", this.backLeftInputs);
        Logger.processInputs("Subsystems/Drive/Module Data/Back Right", this.backRightInputs);

        this.systemState = handleStateTransition();

        Logger.recordOutput("Subsystems/Drive/SystemState", this.systemState);
        Logger.recordOutput("Subsystems/Drive/DesiredState",this.wantedState);
        this.applyStates();
    }

    private DriveState handleStateTransition() {
        return switch (this.wantedState) {
        case SYS_ID -> DriveState.SYS_ID;
        case TELEOP_DRIVE -> DriveState.TELEOP_DRIVE;
        case CHOREO_PATH -> {
            if (this.systemState != DriveState.CHOREO_PATH) {
                this.choreoTimer.restart();
            }
            this.choreoSampleToBeApplied = this.desiredChoreoTrajectory.sampleAt(this.choreoTimer.get(), false);
            yield DriveState.CHOREO_PATH;
        }
        case ROTATION_LOCK -> DriveState.ROTATION_LOCK;
        case DRIVE_TO_POINT -> DriveState.DRIVE_TO_POINT;
        case IDLE -> {
            if (this.systemState != DriveState.IDLE) {
                this.disableMotors();
            }
            yield DriveState.IDLE;
        }
        default -> throw new IllegalArgumentException("Wanted state is not valid.");
        };
    }

    private void applyStates() {
        switch (this.systemState) {
        default:
        case IDLE:
        case SYS_ID:
            break;
        case TELEOP_DRIVE:
            this.io.setSwerveState(new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(new ChassisSpeeds(0,0,0))
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)); //change to CloseLoopVoltage idk where is that.
            break;
        case CHOREO_PATH: {
            if (this.choreoSampleToBeApplied.isPresent()) {
                var sample = this.choreoSampleToBeApplied.get();
                Logger.recordOutput("Subsystems/Drive/Choreo/Timer Value", this.choreoTimer.get());
                Logger.recordOutput("Subsystems/Drive/Choreo/Traj Name", this.desiredChoreoTrajectory.name());
                Logger.recordOutput("Subsystems/Drive/Choreo/Total time", this.desiredChoreoTrajectory.getTotalTime());
                Logger.recordOutput("Subsystems/Drive/Choreo/sample/Desired Pose", sample.getPose());
                Logger.recordOutput(
                        "Subsystems/Drive/Choreo/sample/Desired Chassis Speeds", sample.getChassisSpeeds());
                Logger.recordOutput("Subsystems/Drive/Choreo/sample/Module Forces X", sample.moduleForcesX());
                Logger.recordOutput("Subsystems/Drive/Choreo/sample/Module Forces Y", sample.moduleForcesY());
                synchronized (this.swerveInputs) {
                    var pose = this.swerveInputs.Pose;

                    var targetSpeeds = sample.getChassisSpeeds();
                    targetSpeeds.vxMetersPerSecond += this.choreoXController.calculate(pose.getX(), sample.x);
                    targetSpeeds.vyMetersPerSecond += this.choreoYController.calculate(pose.getY(), sample.y);
                    targetSpeeds.omegaRadiansPerSecond += this.choreoThetaController.calculate(
                            pose.getRotation().getRadians(), sample.heading);

                    this.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                            .withSpeeds(targetSpeeds)
                            .withWheelForceFeedforwardsX(sample.moduleForcesX())
                            .withWheelForceFeedforwardsY(sample.moduleForcesY())
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
                }
            }
            break;
        }
        case ROTATION_LOCK:
            this.io.setSwerveState(this.driveAtAngle
                    .withVelocityX(this.calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
                    .withVelocityY(this.calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond)
                    .withTargetDirection(this.desiredRotationForRotationLockState));
            break;
        case DRIVE_TO_POINT:
            Translation2d translationToDesiredPoint = this.desiredPoseForDriveToPoint.getTranslation().minus(this.swerveInputs.Pose.getTranslation());
            var linearDistance = translationToDesiredPoint.getNorm();
            var frictionConstant = 0.0;
            if (linearDistance >= Units.inchesToMeters(0.5)) {
                frictionConstant = DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * this.maxVelocity;
            }
            var directionOfTravel = translationToDesiredPoint.getAngle();
            var velocityOutput = 0.0;
            if (DriverStation.isAutonomous()) {
                velocityOutput = Math.min(
                        Math.abs(this.autoDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                        this.maxVelocityOutputForDriveToPoint);
            } else {
                velocityOutput = Math.min(
                        Math.abs(this.teleopDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                        this.maxVelocityOutputForDriveToPoint);
            }
            var xComponent = velocityOutput * directionOfTravel.getCos();
            var yComponent = velocityOutput * directionOfTravel.getSin();

            Logger.recordOutput("Subsystems/Drive/DriveToPoint/xVelocitySetpoint", xComponent);
            Logger.recordOutput("Subsystems/Drive/DriveToPoint/yVelocitySetpoint", yComponent);
            Logger.recordOutput("Subsystems/Drive/DriveToPoint/velocityOutput", velocityOutput);
            Logger.recordOutput("Subsystems/Drive/DriveToPoint/linearDistance", linearDistance);
            Logger.recordOutput("Subsystems/Drive/DriveToPoint/directionOfTravel", directionOfTravel);
            Logger.recordOutput("Subsystems/Drive/DriveToPoint/desiredPoint", this.desiredPoseForDriveToPoint);

            if (Double.isNaN(this.maximumAngularVelocityForDriveToPoint)) {
                this.io.setSwerveState(this.driveAtAngle
                        .withVelocityX(xComponent)
                        .withVelocityY(yComponent)
                        .withTargetDirection(this.desiredPoseForDriveToPoint.getRotation()));
            } else {
                this.io.setSwerveState(this.driveAtAngle
                        .withVelocityX(xComponent)
                        .withVelocityY(yComponent)
                        .withTargetDirection(this.desiredPoseForDriveToPoint.getRotation())
                        .withMaxAbsRotationalRate(this.maximumAngularVelocityForDriveToPoint));
            }
            break;
        }
    }

    @Override
    public void simulationPeriodic() {
        synchronized (this.moduleIOLock) {
            this.io.updateSimState();
        }
    }

    public void setWantedState(DriveState state) {
        this.wantedState = state;
        SmartDashboard.putString("wantedState", this.wantedState.toString());
    }

    public void setDesiredChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
        this.desiredChoreoTrajectory = trajectory;
        this.setWantedState(DriveState.DRIVE_TO_POINT);
        this.choreoTimer.reset();
    }

    public void setDesiredPoseForDriveToPoint(Pose2d pose) {
        this.desiredPoseForDriveToPoint = pose;
        this.setWantedState(DriveState.DRIVE_TO_POINT);
        this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
        this.maximumAngularVelocityForDriveToPoint = Double.NaN;
    }

    public void setDesiredPoseForDriveToPointWithMaximumAngularVelocity(
            Pose2d pose, double maximumAngularVelocityForDriveToPoint) {
        this.desiredPoseForDriveToPoint = pose;
        this.setWantedState(DriveState.DRIVE_TO_POINT);
        this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
        this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
    }

    public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutputForDriveToPoint) {
        this.desiredPoseForDriveToPoint = pose;
        this.setWantedState(DriveState.DRIVE_TO_POINT);
        this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
    }

    public void setDesiredPoseForDriveToPointWithConstraints(
            Pose2d pose, double maxVelocityOutputForDriveToPoint, double maximumAngularVelocityForDriveToPoint) {
        this.desiredPoseForDriveToPoint = pose;
        this.setWantedState(DriveState.DRIVE_TO_POINT);
        this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
        this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint;
    }

    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = MathUtil.applyDeadband(this.controller.getLeftY(), CONTROLLER_DEADBAND);
        double yMagnitude = MathUtil.applyDeadband(this.controller.getLeftX(), CONTROLLER_DEADBAND);
        SmartDashboard.putNumber("xMagnitude", xMagnitude);
        SmartDashboard.putNumber("yMagnitude", yMagnitude);
        double angularMagnitude = MathUtil.applyDeadband(this.controller.getRightX(), CONTROLLER_DEADBAND);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (FieldUtil.isBlueAlliance() ? -xMagnitude * this.maxVelocity : xMagnitude * this.maxVelocity) * this.teleopVelocityCoefficient;
        double yVelocity = (FieldUtil.isBlueAlliance() ? -yMagnitude * this.maxVelocity : yMagnitude * this.maxVelocity) * this.teleopVelocityCoefficient;
        double angularVelocity = angularMagnitude * this.maxAngularVelocity * this.rotationVelocityCoefficient;
        SmartDashboard.putNumber("xVelocity", xVelocity);
        SmartDashboard.putNumber("yVelocity", yVelocity);
        SmartDashboard.putNumber("angularVelocity", angularVelocity);

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(this.swerveInputs.Speeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SCALAR);
        
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), this.swerveInputs.Pose.getRotation()),
                this.swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
    }

    public void resetTranslationAndRotation(Pose2d pose2d) {
        resetTranslation(pose2d);
        resetRotation(pose2d.getRotation());
    }

    public void resetRotation(Rotation2d rotation2d) {
        this.io.resetToParamaterizedRotation(rotation2d);
    }

    public void resetTranslation(Pose2d pose) {
        this.io.resetRobotTranslation(pose.getTranslation());
    }

    public void resetRotationBasedOnAlliance() {
        this.io.resetRotation();
    }

    public boolean isAtDriveToPointSetpoint() {
        var distance = this.desiredPoseForDriveToPoint
                .getTranslation()
                .minus(this.swerveInputs.Pose.getTranslation())
                .getNorm();
        Logger.recordOutput("Subsystems/Drive/DriveToPoint/distanceFromEndpoint", distance);
        return MathUtil.isNear(0.0, distance, TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS_DRIVE_TO_POINT);
    }

    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    public boolean isAtDesiredRotation(double tolerance) {
        return this.driveAtAngle.HeadingController.getPositionError() < tolerance;
    }

    public boolean isAtChoreoSetpoint() {
        if (this.systemState != DriveState.CHOREO_PATH) {
            return false;
        }
        return MathUtil.isNear(
            this.desiredChoreoTrajectory.getFinalPose(false).get().getX(),
                this.swerveInputs.Pose.getX(),
                TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS) && MathUtil.isNear(
                    this.desiredChoreoTrajectory.getFinalPose(false).get().getY(),
                        this.swerveInputs.Pose.getY(),
                        TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS);
    }

    public boolean isAtEndOfChoreoTrajectoryOrDriveToPoint() {
        if (this.desiredChoreoTrajectory != null) {
            return (MathUtil.isNear(
                this.desiredChoreoTrajectory
                            .getFinalPose(false)
                            .get()
                            .getX(),
                    this.swerveInputs.Pose.getX(),
                    TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS) && MathUtil.isNear(
                        this.desiredChoreoTrajectory
                                    .getFinalPose(false)
                                    .get()
                                    .getY(),
                            this.swerveInputs.Pose.getY(),
                            TRANSLATION_ERROR_MARGIN_FOR_RELEASING_PIECE_METERS)) || isAtDriveToPointSetpoint();
        } else {
            return isAtDriveToPointSetpoint();
        }
    }

    public double getRobotDistanceFromChoreoEndpoint() {
        Logger.recordOutput(
                "Choreo/FinalPose", this.desiredChoreoTrajectory.getFinalPose(false).get());
        var distance = Math.abs(this.desiredChoreoTrajectory
                .getFinalPose(false)
                .get()
                .minus(robotToFieldFromSwerveDriveOdometry)
                .getTranslation()
                .getNorm());
        Logger.recordOutput("Choreo/DistanceFromEndpoint", distance);
        return distance;
    }

    public double getDistanceFromDriveToPointSetpoint() {
        var diff = this.desiredPoseForDriveToPoint
                .getTranslation()
                .minus(this.swerveInputs.Pose.getTranslation())
                .getNorm();
        Logger.recordOutput("DistanceFromDriveToPointSetpoint", diff);
        return diff;
    }

    public void setTeleopVelocityCoefficient(double teleopVelocityCoefficient) {
        this.teleopVelocityCoefficient = teleopVelocityCoefficient;
    }

    public void setRotationVelocityCoefficient(double rotationVelocityCoefficient) {
        this.rotationVelocityCoefficient = rotationVelocityCoefficient;
    }

    public SwerveIOCTRE getIO() {
        return this.io;
    }

    public static void setRobotToFieldFromSwerveDriveOdometry(Pose2d pose) {
        robotToFieldFromSwerveDriveOdometry = pose;
    }

    public void disableMotors() {
        this.io.disableMotors();
    }

    public void disableMotorsWithState() {
        this.io.disableMotors();
        
        //Allow for the method to be called immediately and not wait for the next cycle.
        this.setWantedState(DriveState.TELEOP_DRIVE);
        this.systemState = DriveState.TELEOP_DRIVE;//We allready disabled the motors, so we can set the state to IDLE immediately.
    }


}