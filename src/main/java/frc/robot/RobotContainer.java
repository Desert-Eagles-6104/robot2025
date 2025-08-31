// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib25.Subsystems.Drive.SwerveIOCTRE;
import frc.DELib25.Subsystems.Drive.SwerveSubsystem;
import frc.DELib25.Subsystems.Drive.SwerveUtil.SwerveConstants;
import frc.DELib25.Subsystems.Pose.PoseTracker;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Util.DriverStationController;
import frc.DELib25.Util.MacAddressUtil;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystemRobot2025;
import frc.robot.Commands.integrationCommands.Human;
import frc.robot.Commands.integrationCommands.L2Score;
import frc.robot.Commands.integrationCommands.L3Score;
import frc.robot.Commands.integrationCommands.L4Score;
import frc.robot.Commands.integrationCommands.ResetAllSubsystems;
import frc.robot.Commands.integrationCommands.SmartPreset;
import frc.robot.constants.TrainingChassisSwerveConstants;
import frc.robot.constants.SwerveConstantsRobotSeason;
import frc.robot.presetState.PresetState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private CommandPS5Controller drivercontroller = new CommandPS5Controller(0);
	private CommandPS5Controller operatorController;// = new CommandPS5Controller(1);
	private DriverStationController driverStationController = new DriverStationController(2);
	private final SwerveSubsystem swerveSubsystem;
	private final VisionSubsystemRobot2025 m_vision;
	private Climb m_climb;
	private ElevatorSubsystem m_elevator;
	private GripperArmSubsystem m_gripperArm;
	private GripperSubsystem m_gripper;
	private Gripper2Subsystem m_gripper2;
	public static BooleanSupplier m_isLocalisation = () -> false;
	public static BooleanSupplier m_isLocalisationOmega = () -> false;
	public static PresetState m_state = PresetState.Home;

	public RobotContainer() {
		SwerveConstants swerveConstants = this.getSwerveConstants();
		SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants = swerveConstants.getSwerveModuleConstants();
		this.swerveSubsystem = new SwerveSubsystem(
				new SwerveIOCTRE(swerveConstants.getSwerveDrivetrainConstants(), moduleConstants),
				drivercontroller,
				moduleConstants[0].SpeedAt12Volts,
				moduleConstants[0].SpeedAt12Volts / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY),
				swerveConstants.getTranslationSysIdConfig(),
				swerveConstants.getRotationSysIdConfig(),
				swerveConstants.getSteerSysIdConfig()
			);
		

		// m_elevator = new
		// ElevatorSubsystem(Constants.Elevator.ElevatorConfiguration);
		// m_gripperArm = new
		// GripperArmSubsystem(Constants.GripperArm.configuration);
		// m_climb = new Climb();
		// m_gripper = new GripperSubsystem();
		// m_gripper2 = new Gripper2Subsystem();
		m_vision = new VisionSubsystemRobot2025(new CameraSettings(0.20449, 0.20083, 0.57226, 13.18, 21.18, 15.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
		// m_poseEstimator = new PoseEstimatorSubsystem(this.swerveSubsystem,
		// m_vision);
		m_isLocalisation = driverStationController.LeftSwitch().negate();
		m_isLocalisationOmega = driverStationController.LeftMidSwitch().negate();

		// controls
		dashboardResets();
		// SwerveBinding();
		// auto();
		// DriverManuals();
		// OperatorManuals();

		// drivercontroller.R3().whileTrue(new InstantCommand(() ->
		// m_vision.getCurrentID()));

	}

	public void dashboardResets() {
		SmartDashboard.putData("Reset Odometry From Limelight", new InstantCommand(() -> {
			Pose2d pose = PoseTracker.getInstance().getLimelightMeasurement().pose;
			if (pose != null) {
				this.swerveSubsystem.getIO().resetPose(pose);
			}
		}));
		SmartDashboard.putData("Reset Elevator", new InstantCommand(() -> m_elevator.resetPosition(0)).ignoringDisable(true));
		SmartDashboard.putData("Reset Gripper Arm", new InstantCommand(() -> m_gripperArm.resetPosition(37)).ignoringDisable(true));
		SmartDashboard.putData("Set Elevator Coast", new InstantCommand(() -> m_elevator.changeNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
		SmartDashboard.putData("Set Elevator Brake", new InstantCommand(() -> m_elevator.changeNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
		// SmartDashboard.putNumber("AprilTagID", VisionSubsystem.getID());
	}

	public void disableMotors() {
		this.swerveSubsystem.disableMotors();
	}

	// TODO: Phase this out
	public VisionSubsystemRobot2025 getVision() {
		return m_vision;
	}

	public void SwerveBinding() {
		// this.drivercontroller.L3().toggleOnTrue()
		// drivercontroller.R3().toggleOnTrue(new InstantCommand(() ->
		// ReefUtill.Update(drivercontroller.R3())));
	}

	public void OperatorManuals() {
		operatorController.square().onTrue(new L4Score(m_elevator, m_gripperArm, m_gripper, m_state, m_gripper2, operatorController.L3()));
		operatorController.triangle().onTrue(new L3Score(m_elevator, m_gripperArm, m_gripper, m_state, m_gripper2, operatorController.L3()));
		operatorController.circle().onTrue(new L2Score(m_elevator, m_gripperArm, m_gripper, m_state, m_gripper2, operatorController.L3()));
		operatorController.R3().onTrue(new Human(m_elevator, m_gripperArm, m_gripper, m_state, m_gripper2));
		operatorController.povDown().onTrue(new ResetAllSubsystems(m_elevator, m_gripperArm));
		operatorController.R2().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, PresetState.AlgeL2));
		operatorController.L2().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, PresetState.AlgeL3));
	}

	public void DriverManuals() {
		// drivercontroller.L2().whileTrue(new GripperSet(m_gripper2, 0.6));
		// drivercontroller.R2().whileTrue(new GripperSet(m_gripper2, -0.6));
		// drivercontroller.povLeft().whileTrue(new SetPercent(m_climb, -0.65));
		// drivercontroller.povRight().whileTrue(new SetPercent(m_climb, 0.65));
		// drivercontroller.square().onTrue(new L4Score(m_elevator,
		// m_gripperArm, m_gripper, m_state,
		// m_gripper2,drivercontroller.cross()));
		// drivercontroller.triangle().onTrue(new L3Score(m_elevator,
		// m_gripperArm, m_gripper, m_state,
		// m_gripper2,drivercontroller.cross()));
		// drivercontroller.circle().onTrue(new L2Score(m_elevator,
		// m_gripperArm, m_gripper, m_state,
		// m_gripper2,drivercontroller.cross()));
		// drivercontroller.R3().onTrue(new Human(m_elevator, m_gripperArm,
		// m_gripper, m_state, m_gripper2));
		// drivercontroller.povDown().onTrue(new ResetAllSubsystems(m_elevator,
		// m_gripperArm));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * 
	 * @return the command to run in autonomous \\/ public Command
	 * getAutonomousCommand() { return m_swerveAutoBuilder.getAuto(); }
	 */

	public void auto() {}

	public SwerveSubsystem getSwerveSubsystem() {
		return this.swerveSubsystem;
	}

	// if you change the mac address (or switch the radio) update the constants
	// file with the new mac address
	public SwerveConstants getSwerveConstants() {
		switch (MacAddressUtil.getMACAddress()) {
		case SwerveConstantsRobotSeason.MAC_ADDRESS:
			return new SwerveConstantsRobotSeason();
		case TrainingChassisSwerveConstants.MAC_ADDRESS:
			return new TrainingChassisSwerveConstants();
		default:
			throw new IllegalArgumentException("Unknown MAC Address: " + MacAddressUtil.getMACAddress());
		}
	}

}