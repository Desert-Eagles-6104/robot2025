// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Util;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class SwerveAutoBuilder {
    private SwerveSubsystem m_swerve;
    private final SendableChooser<Command> m_autoChooser;
    private HashMap<String, Command> m_fullAutoCommands;
    private BooleanSupplier m_rotationOverrideConditionSupplier;
    private DoubleSupplier m_rotationTargetSupplier;
    private RobotConfig m_robotConfig;

    public SwerveAutoBuilder(SwerveSubsystem swerve){
        m_swerve = swerve;
        m_fullAutoCommands = new HashMap<String, Command>();
        try{
            m_robotConfig = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
          }
        AutoBuilder.configure(
            m_swerve::getPose, // Robot pose supplier
            m_swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            m_swerve::getRobotRelativeVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(6.0, 0.0, 0.0) // Rotation PID constants
                ),
                m_robotConfig,
               () -> { 
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                 return false;
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
               },
                m_swerve // Reference to this subsystem to set requirements
        );

        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.setDefaultOption("Default", new PrintCommand("Default"));
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
        m_rotationOverrideConditionSupplier = () -> false;
        m_rotationTargetSupplier = () -> 0;

        NamedCommands.registerCommand("EnableRotationOverride", new InstantCommand(() ->setRotationOverrideConditionSupplier(() -> true)));
        NamedCommands.registerCommand("DisableRotationOverride", new InstantCommand(() ->setRotationOverrideConditionSupplier(() -> false)));}

    /**
     * first add all commands 
     * @param name
     * @param command
     */
    public void addCommand(String name, Command command){
        NamedCommands.registerCommand(name, command);
    }

    /**
     * adds all the commands 
     * @param name
     * @param commands
     */
    public void addCommand(String[] name, Command... commands){
        int i = 0;
        for(Command command : commands){
            NamedCommands.registerCommand(name[i], command);            
            i++;
        }
    }

    /**
     * call this fnction only after all commands were added
     */
    public void buildAutos(){
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        for (String autoName : autoNames) {
            m_fullAutoCommands.put(autoName, AutoBuilder.buildAuto(autoName));
        }
    }
    
    /**
     * return the chosen auto command
     * @return
     */
    public Command getAuto(){
        return AutoBuilder.buildAuto(getAutoName());        
    }

    public String getAutoName(){
        return m_autoChooser.getSelected().getName();
    } 

    // Use the PathPlannerAuto class to get a path group from an auto




    public void setRotationTargetSupplier(DoubleSupplier rotationTargetSupplier) {
        m_rotationTargetSupplier = rotationTargetSupplier;
        PPHolonomicDriveController.setRotationTargetOverride(() -> getRotationTargetOverride(m_rotationOverrideConditionSupplier, rotationTargetSupplier));
    }

    public void setRotationOverrideConditionSupplier(BooleanSupplier rotationOverrideConditionSupplier) {
        m_rotationOverrideConditionSupplier = rotationOverrideConditionSupplier;
        PPHolonomicDriveController.setRotationTargetOverride(() -> getRotationTargetOverride(rotationOverrideConditionSupplier, m_rotationTargetSupplier));
    }
    
    public Optional<Rotation2d> getRotationTargetOverride(BooleanSupplier condition, DoubleSupplier target){
        if(condition.getAsBoolean()){
            return Optional.of(Rotation2d.fromDegrees(target.getAsDouble()));
        }
        else{
            return Optional.empty();
        }
    }

    private void autoDrive(ChassisSpeeds chassisSpeeds){
        m_swerve.drive(chassisSpeeds, false, false, new Translation2d());;
    }
}