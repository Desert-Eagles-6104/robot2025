package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.VisionSubsystemRobot2025;

public class DriveAssist {    
    private double m_kp;
    private BooleanSupplier m_intakeButton;
    private LinearFilter m_filter;
    private VisionSubsystemRobot2025 vision;

    public DriveAssist(VisionSubsystemRobot2025 vision, double kp, BooleanSupplier intakeButton) {
        this.vision = vision;
        m_kp = kp;
        m_intakeButton = intakeButton;
        m_filter = LinearFilter.movingAverage(4);
    }

    public ChassisSpeeds update(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading){
        ChassisSpeeds toReturn = new ChassisSpeeds();
        if(this.vision.getTvNote() && m_intakeButton.getAsBoolean()){
            if(Robot.s_Alliance == Alliance.Red){
                toReturn = ChassisSpeeds.fromRobotRelativeSpeeds(0, -m_filter.calculate(-this.vision.getTxNote())*m_kp, 0, robotHeading);
            }
            else{
                toReturn = ChassisSpeeds.fromRobotRelativeSpeeds(0, m_filter.calculate(-this.vision.getTxNote())*m_kp, 0, robotHeading);
            }
        }
        chassisSpeeds.vxMetersPerSecond = toReturn.vxMetersPerSecond + chassisSpeeds.vxMetersPerSecond;
        chassisSpeeds.vyMetersPerSecond = toReturn.vyMetersPerSecond + chassisSpeeds.vyMetersPerSecond;
        return chassisSpeeds;
    }
}
