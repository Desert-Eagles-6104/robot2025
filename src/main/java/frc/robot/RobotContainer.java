// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveCommands.ResetSwerveModules;
import frc.DELib25.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Sysid.PhoneixSysid;
import frc.DELib25.Util.DriverStationController;
import frc.DELib25.Util.SwerveAutoBuilder;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Commands.IntakeArmCommands.IntakeArmSetPosition;
import frc.robot.Commands.integrationCommands.SmartPreset;
import frc.robot.PresetUtil.PresetState;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since 
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CommandPS5Controller drivercontroller = new CommandPS5Controller(0);
  private CommandPS5Controller operatorController = new CommandPS5Controller(1);
  private DriverStationController driverStationController = new DriverStationController(2);
  private SwerveSubsystem m_swerve;
  private VisionSubsystem m_vision;
  private ElevatorSubsystem m_elevator;
  private GripperArmSubsystem m_gripperArm;
  private GripperSubsystem m_gripper;
  private IntakeArmSubsystem m_intakeArm;
  private PoseEstimatorSubsystem m_poseEstimator;
  private PhoneixSysid m_sysid;
  private SwerveAutoBuilder m_swerveAutoBuilder;
  public static BooleanSupplier m_isLocalisation = ()-> false;
  public static BooleanSupplier m_isLocalisationOmega = () -> false;
  public static PresetState m_state = PresetState.Home;

  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_elevator = new ElevatorSubsystem(Constants.Elevator.ElevatorConfiguration);
    m_gripperArm = new GripperArmSubsystem(Constants.GripperArm.configuration);
    m_intakeArm = new IntakeArmSubsystem(Constants.IntakeArm.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(-0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    m_sysid = new PhoneixSysid(Constants.sysidConfiguration, m_gripperArm);
    m_poseEstimator = new PoseEstimatorSubsystem(m_swerve);
    m_isLocalisation = driverStationController.LeftSwitch().negate();
    m_isLocalisationOmega = driverStationController.LeftMidSwitch().negate();
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1(), drivercontroller.R1()));
    m_swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    // controls
    drivercontroller.circle().onTrue(new IntakeArmSetPosition(m_intakeArm, 15, false));
    drivercontroller.square().onTrue(new IntakeArmSetPosition(m_intakeArm, 90,false));
    drivercontroller.triangle().onTrue(new IntakeArmSetPosition(m_intakeArm, 60, false));
    dashboardResets();
    SwerveBinding();
    auto();
    updateState();
    drivercontroller.R1().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state));
    drivercontroller.L1().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state));
  }

  public void dashboardResets(){
    SmartDashboard.putData("Reset Odometry From Limelight", new InstantCommand(() -> PoseEstimatorSubsystem.resetPositionFromCamera()));
    SmartDashboard.putData("Reset Elevator" , new InstantCommand(() -> m_elevator.resetPosition(0)).ignoringDisable(true));
    SmartDashboard.putData("Reset Gripper Arm" , new InstantCommand(() -> m_gripperArm.resetPosition(37)).ignoringDisable(true));
    SmartDashboard.putData("Reset Intake Arm" ,new InstantCommand(() -> m_intakeArm.resetPosition(0.0)).ignoringDisable(true));
    SmartDashboard.putData("Set Elevator Coast" , new InstantCommand(() -> m_elevator.changeNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
    SmartDashboard.putData("Set Elevator Brake" , new InstantCommand(() -> m_elevator.changeNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
  }

  public void disableMotors() {
    m_swerve.disableModules();
  
  }

  public void updateState(){
    operatorController.cross().onTrue(new InstantCommand(() -> m_state = PresetState.L1));
    operatorController.circle().onTrue(new InstantCommand(() -> m_state = PresetState.L2));
    operatorController.triangle().onTrue(new InstantCommand(() -> m_state = PresetState.L3));
    operatorController.square().onTrue(new InstantCommand(() -> m_state = PresetState.L4));
    operatorController.options().onTrue(new InstantCommand(() -> m_state = PresetState.Home));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_swerveAutoBuilder.getAuto();
    // return Commands.none();
  }

  public void auto(){
    //add commands 

    /* EXAMPLE 
    swerveAutoBuilder.addCommand("InatkeUntilHasNote", new IntakeEatUntilHasNote(m_intake, 0.8, true).withTimeout(2));*/
    
    m_swerveAutoBuilder.buildAutos();
  }

  public void SwerveBinding(){
    SmartDashboard.putData("calibrate Swerve Modules", new ResetSwerveModules(m_swerve).ignoringDisable(true));
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1(), drivercontroller.L2()));
  }
}