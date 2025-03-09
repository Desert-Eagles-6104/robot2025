// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Thread.State;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.DriveAssistAuto;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.DriveAssistToReef;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.ReefAssist;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Sysid.PhoneixSysid;
import frc.DELib25.Util.DriverStationController;
import frc.DELib25.Util.SwerveAutoBuilder;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Commands.SetState;
import frc.robot.Commands.Climb.SetPercent;
import frc.robot.Commands.ElevatorCommands.HomingElevator;
//import frc.robot.Commands.GripperCommands.EatUntilCoral;
import frc.robot.Commands.GripperCommands.GripperSet;
import frc.robot.Commands.GripperCommands.GripperSetPrecent;
import frc.robot.Commands.integrationCommands.ResetAllSubsystems;
import frc.robot.Commands.integrationCommands.SmartPreset;
import frc.robot.presetState.PresetState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;

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
  private Climb m_climb;
  private ElevatorSubsystem m_elevator;
  private GripperArmSubsystem m_gripperArm;
  private GripperSubsystem m_gripper;
  private Gripper2Subsystem m_gripper2;
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
    m_climb = new Climb();
    m_gripper = new GripperSubsystem();
    m_gripper2 = new Gripper2Subsystem();
    m_vision = new VisionSubsystem(new CameraSettings(0.20449, 0.20083, 0.57226 , 13.18, 21.18, 15.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    m_sysid = new PhoneixSysid(Constants.sysidConfiguration, m_gripperArm);
    m_poseEstimator = new PoseEstimatorSubsystem(m_swerve);
    m_isLocalisation = driverStationController.LeftSwitch().negate();
    m_isLocalisationOmega = driverStationController.LeftMidSwitch().negate();
    m_swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);

    // controls
    dashboardResets();
    SwerveBinding();
    auto();

    // drivercontroller.povDown().onTrue(new HomingElevator(m_elevator));
    drivercontroller.L2().whileTrue(new GripperSet(m_gripper2, -0.7));
    drivercontroller.R2().whileTrue(new GripperSet(m_gripper2, 0.6));
    drivercontroller.povLeft().whileTrue(new SetPercent(m_climb, -0.2));
    drivercontroller.povRight().whileTrue(new SetPercent(m_climb, 0.2));
    
    
    //operator
    operatorController.circle().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L2));
    operatorController.triangle().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L3));
    operatorController.square().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L4));
    operatorController.R3().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.Human));
    operatorController.povDown().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.Home));
    operatorController.options().onTrue(new ResetAllSubsystems(m_elevator, m_gripperArm));
    
    
    // operatorController.cross().onTrue( new SetState(presetState.getPresetState(PresetState.Home)));
    // operatorController.square().onTrue( new SetState(presetState.getPresetState(PresetState.L4)));

    // Arab Score&intake
    drivercontroller.circle().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L2));
    drivercontroller.triangle().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L3));
    drivercontroller.square().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L4));
    //drivercontroller.R3().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.Human));
    drivercontroller.povDown().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.Home));
    drivercontroller.options().onTrue(new ResetAllSubsystems(m_elevator, m_gripperArm));
    
    // SCORE AND INTAKE&
   // drivercontroller.R1().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state));
   // drivercontroller.L1().onTrue(new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state));
   // drivercontroller.R1().debounce(0.4).and(() -> m_gripper.HasGamePiece()).whileTrue(new GripperSetPrecent(m_gripper2, 0.4).andThen(new GripperSetPrecent(m_gripper2, 0.0)));
   // drivercontroller.R1().debounce(0.4).and(() -> !m_gripper.HasGamePiece()).whileTrue(new GripperSetPrecent(m_gripper2, -0.4).andThen(new GripperSetPrecent(m_gripper2, 0.0)));
   // drivercontroller.L1().debounce(0.4).and(() -> m_gripper.HasGamePiece()).whileTrue(new GripperSetPrecent(m_gripper2, 0.4).andThen(new GripperSetPrecent(m_gripper2, 0.0)));
   // drivercontroller.L1().debounce(0.4).and(() -> !m_gripper.HasGamePiece()).whileTrue(new GripperSetPrecent(m_gripper2, -0.4).andThen(new GripperSetPrecent(m_gripper2, 0.0)));
  }

  public void dashboardResets(){
    SmartDashboard.putData("Reset Odometry From Limelight", new InstantCommand(() -> PoseEstimatorSubsystem.resetPositionFromCamera()));
    SmartDashboard.putData("Reset Elevator" , new InstantCommand(() -> m_elevator.resetPosition(0)).ignoringDisable(true));
    SmartDashboard.putData("Reset Gripper Arm" , new InstantCommand(() -> m_gripperArm.resetPosition(37)).ignoringDisable(true));
    SmartDashboard.putData("Set Elevator Coast" , new InstantCommand(() -> m_elevator.changeNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
    SmartDashboard.putData("Set Elevator Brake" , new InstantCommand(() -> m_elevator.changeNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
    // SmartDashboard.putNumber("AprilTagID", VisionSubsystem.getID());    
  }

  public void disableMotors() {
    m_swerve.disableModules();
    
  }
  
    public void SwerveBinding(){
      drivercontroller.PS().onTrue(new ReefAssist(m_swerve));
      m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1().or(drivercontroller.L1()), drivercontroller.R1().and(drivercontroller.L1().negate()), drivercontroller.L1().and(drivercontroller.R1().negate())));
    }

  public Command getAuto() {
    return m_swerveAutoBuilder.getAuto();
  }

  public void DriverResets(){
    drivercontroller.povUp().onTrue(new ResetAllSubsystems(m_elevator, m_gripperArm));
    drivercontroller.L2().whileTrue(new GripperSet(m_gripper2, -0.7));
    drivercontroller.R2().whileTrue(new GripperSet(m_gripper2, 0.6));
    drivercontroller.povLeft().whileTrue(new SetPercent(m_climb, -0.25));
    drivercontroller.povRight().whileTrue(new SetPercent(m_climb, 0.25));
    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_swerveAutoBuilder.getAuto();
  }

  public void auto(){
    //add commands 
    // m_swerveAutoBuilder.addCommand("l4", new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L4));
    // m_swerveAutoBuilder.addCommand("l3", new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L3));
    // m_swerveAutoBuilder.addCommand("l2", new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L2));
    // m_swerveAutoBuilder.addCommand("l1", new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.AlgeL2));
    // m_swerveAutoBuilder.addCommand("human", new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.Human));
    m_swerveAutoBuilder.addCommand("Score", new GripperSet(m_gripper2, 0.5));
    //  m_swerveAutoBuilder.addCommand("IntakeUntilHasCoral", new EatUntilCoral(m_gripper2, 0.5));
    /* EXAMPLE 
    swerveAutoBuilder.addCommand("InatkeUntilHasNote", new IntakeEatUntilHasNote(m_intake, 0.8, true).withTimeout(2));*/
    
    m_swerveAutoBuilder.buildAutos();
  }
}