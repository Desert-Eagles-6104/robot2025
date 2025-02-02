// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
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
import frc.DELib25.Util.DriverStationController;
// import frc.DELib25.Util.SwerveAutoBuilder; 
import frc.robot.commands.Shit;
import frc.robot.subsystems.ExampleSubsystem;

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
  private PoseEstimatorSubsystem m_poseEstimator;
  private ExampleSubsystem m_ExampleSubsystem;
  // 
  //private SwerveAutoBuilder swerveAutoBuilder;
  public static BooleanSupplier m_isLocalisation = ()-> false;
  public static BooleanSupplier m_isLocalisationOmega = () -> false;
  


  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_vision = new VisionSubsystem(new CameraSettings(-0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    // swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    m_ExampleSubsystem = new ExampleSubsystem();
    m_poseEstimator = new PoseEstimatorSubsystem(m_swerve);
    m_isLocalisation = driverStationController.LeftSwitch().negate();
    m_isLocalisationOmega = driverStationController.LeftMidSwitch().negate();
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1(), drivercontroller.L2()));
    SmartDashboard.putData("reset Odometry from limelight", new InstantCommand(() -> PoseEstimatorSubsystem.resetPositionFromCamera()));
    SwerveBinding();
    presets();
    resets();
    auto();
    driverStationController.LeftBlue().onTrue(new RotateToTarget(m_swerve));
    drivercontroller.L2().onTrue(new Shit(m_ExampleSubsystem, 0.5));
    drivercontroller.R2().onTrue(new Shit(m_ExampleSubsystem,-0.5));
  }

  public void disableMotors() {
    m_swerve.disableModules();
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return swerveAutoBuilder.getAuto();
    return null;
  }

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