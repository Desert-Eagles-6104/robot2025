// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;

/** Add your docs here. */
public class ReefUtill {
    private Translation2d m_pointLeft;
    private Translation2d m_pointRight;
    private Rotation2d m_robotAngle;
    private static final double fieldLength = 17.54825;

    public ReefUtill(Translation2d pointLeft, Translation2d pointRight ,Rotation2d robotAngle){
        m_pointLeft = pointLeft;
        m_pointRight = pointRight;
        m_robotAngle = robotAngle;
    }
    
    public enum ReefFace{
        F1,
        F2,
        F3,
        F4,
        F5,
        F6,
    }

    public Rotation2d getRobotAngleToFace() {
        return m_robotAngle;
    }

    public Translation2d getPointRight() {
        return m_pointRight;
    }

    public Translation2d getPointLeft() {
        return m_pointLeft;
    }

    public static ReefFace getFaceFromVision(){

        int id = (int)VisionSubsystem.getID();//TODO Chnage frim switch to ifs
        switch (id) {
            case 17:
                return ReefFace.F6;
            case 18:
                return ReefFace.F1;
            case 19:
                return ReefFace.F2;
            case 20:
                return ReefFace.F3;
            case 21:
                return ReefFace.F4;
            case 22:
                return ReefFace.F5;
            case 6:
                return ReefFace.F6;
            case 7:
                return ReefFace.F1;
            case 8:
                return ReefFace.F2;
            case 9:
                return ReefFace.F3;
            case 10:
                return ReefFace.F4;
            case 11:
                return ReefFace.F5;
            default:
                return ReefFace.F1;
        }
    }

    public static ReefUtill getReefFacePoint(ReefFace state){
        switch (state) {

            case F1:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 3.18, 4.2), new Translation2d(fieldLength - 3.18, 3.87), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(3.18, 4.2), new Translation2d(3.18, 3.87), Rotation2d.fromDegrees(0.0));
                }
            case F2:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 3.97, 5.23), new Translation2d(fieldLength - 3.69 , 5.07), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(3.97, 5.23), new Translation2d(3.69 , 5.07), Rotation2d.fromDegrees(0.0));
                }

            case F3:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 5.26, 5.07), new Translation2d(fieldLength - 4.98, 5.23), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(5.26, 5.07), new Translation2d(4.98, 5.23), Rotation2d.fromDegrees(0.0));
                }

            case F4:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 5.77, 3.87), new Translation2d(fieldLength - 5.77, 4.2), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(5.77, 3.87), new Translation2d(5.77, 4.2), Rotation2d.fromDegrees(0.0));
                }

            case F5:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 4.98 , 2.83), new Translation2d(fieldLength - 5.26 , 3), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(4.98 , 2.83), new Translation2d(5.26 , 3), Rotation2d.fromDegrees(0.0));
                }

            case F6:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 3.69, 3.0), new Translation2d(fieldLength - 3.97, 2.83), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(3.69, 3.0), new Translation2d(3.97, 2.83), Rotation2d.fromDegrees(0.0));
                }

            default:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 0.0, 0.0), new Translation2d(fieldLength - 0.0, 0.0), Rotation2d.fromDegrees(0.0).rotateBy(Rotation2d.k180deg));
                }
                else{
                    return new ReefUtill(new Translation2d(0.0, 0.0), new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
                }
         }
    }
}

