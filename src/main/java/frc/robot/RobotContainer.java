// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveCommands.ResetSwerveModules;
import frc.DELib25.Subsystems.Swerve.SwerveCommands.RotateToTarget;
import frc.DELib25.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Sysid.PhoneixSysid;
import frc.DELib25.Util.DriverStationController;
import frc.robot.Commands.CoralArmCommands.CoralArmPercent;
import frc.robot.Commands.ElevatorCommands.ElevatorChangeNeutralMode;
import frc.robot.Commands.ElevatorCommands.ElevatorSetPercent;
import frc.robot.Commands.ElevatorCommands.ElevatorSetPosition;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
// import frc.DELib25.Util.SwerveAutoBuilder; 
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
  private CoralArmSubsystem m_CoralArm;
  private AlgaeArmSubsystem m_AlgaeArm;
  private PoseEstimatorSubsystem m_poseEstimator;
  private PhoneixSysid m_FloorPhoneixSysid;
  // 
  //private SwerveAutoBuilder swerveAutoBuilder;
  public static BooleanSupplier m_isLocalisation = ()-> false;
  public static BooleanSupplier m_isLocalisationOmega = () -> false;

  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_elevator = new ElevatorSubsystem(Constants.Elevator.ElevatorConfiguration);
    m_AlgaeArm = new AlgaeArmSubsystem(Constants.AlgaeArm.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(-0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    m_FloorPhoneixSysid = new PhoneixSysid(Constants.sysidConfiguration, m_AlgaeArm);
    // swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    m_poseEstimator = new PoseEstimatorSubsystem(m_swerve);
    m_isLocalisation = driverStationController.LeftSwitch().negate();
    m_isLocalisationOmega = driverStationController.LeftMidSwitch().negate();
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1(), drivercontroller.L2()));
    SmartDashboard.putData("reset Odometry from limelight", new InstantCommand(() -> PoseEstimatorSubsystem.resetPositionFromCamera()));
    SmartDashboard.putData("resetalgaePosition" , new InstantCommand(() -> m_AlgaeArm.resetPosition(0)));
    // drivercontroller.cross().onTrue(new ElevatorSetPosition(m_elevator, 600, false));
    // drivercontroller.povUp().onTrue(new ElevatorSetPosition(m_elevator, 400, false));
    // drivercontroller.povDown().onTrue(new ElevatorSetPosition(m_elevator, 200, false));
    // drivercontroller.povLeft().onTrue(new ElevatorSetPosition(m_elevator, 0, false));
    // SmartDashboard.putData("sofrLimit",new InstantCommand(() -> m_elevator.ControlSoftLimit(false)));
    drivercontroller.circle().toggleOnTrue(new ElevatorSetPercent(m_elevator, 0.05));
    drivercontroller.povDown().onTrue(new CoralArmPercent(m_CoralArm, -0.3));
    drivercontroller.povUp().onTrue(new CoralArmPercent(m_CoralArm, 0.3));

    
    SwerveBinding();
    drivercontroller.square().onTrue( new InstantCommand(() -> m_AlgaeArm.disableMotors()));
    drivercontroller.triangle().onTrue(m_FloorPhoneixSysid.runFullCharacterization(true));
    SmartDashboard.putData("coast" , new InstantCommand(() -> m_AlgaeArm.changeNeutralMode(NeutralModeValue.Coast)));
    presets();
    resets();
    auto();
    driverStationController.LeftBlue().onTrue(new RotateToTarget(m_swerve));
    
  }

  public void disableMotors() {
    m_swerve.disableModules();
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // return swerveAutoBuilder.getAuto();
  //   return 0;
  // }

  public void auto(){
    //add commands 

    /* EXAMPLE 
    swerveAutoBuilder.addCommand("InatkeUntilHasNote", new IntakeEatUntilHasNote(m_intake, 0.8, true).withTimeout(2));*/
    
    // swerveAutoBuilder.buildAutos();
  }

  public void SwerveBinding(){
    SmartDashboard.putData("calibrate Swerve Modules", new ResetSwerveModules(m_swerve).ignoringDisable(true));
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1(), drivercontroller.L2()));
  }
 
  public void presets(){
  
  }

  public void resets(){
   
  }
}