// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;

/** Add your docs here. */
public class ReefUtill extends SubsystemBase {
    private  static ReefFace lastFace = ReefFace.F1;
    private Translation2d m_pointLeft;
    private Translation2d m_pointRight;
    private Rotation2d m_robotAngle;
    private static double m_currentID;
    private static boolean m_bool = false;
    private static double m_id = 0;
            private static final double fieldLength = 17.54825;
        
            public ReefUtill(Translation2d pointLeft, Translation2d pointRight ,Rotation2d robotAngle , boolean bool){
                m_pointLeft = pointLeft;
                m_pointRight = pointRight;
                m_robotAngle = robotAngle;
                m_bool = bool;
            }
            
            public enum ReefFace{
                F1,
                F2,
                F3,
                F4,
                F5,
                F6,
                HR,
                HL
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
        
            @Override
          public void periodic() {
            m_currentID = VisionSubsystem.getCurrentID();
          }
          
        //   public static double getLockedID(){
        //         double id = m_currentID;
        //         SmartDashboard.putNumber("RealID", id);
        //             return id;
        //       }

          public static void Update(BooleanSupplier bool){
            if (bool.getAsBoolean()){
                 m_id = m_currentID;
                SmartDashboard.putNumber("RealID", m_id);
                m_bool =  bool.getAsBoolean();
            }
          }
          
            public static ReefFace getFaceFromVision(){
          
                      // double id = VisionSubsystem.getID();//TODO add && to ifs
            if (m_bool){
                
        if(m_id == 17){
        lastFace = ReefFace.F6;
        return ReefFace.F6;
        }

        else if(m_id == 18){
            lastFace = ReefFace.F1;

            return ReefFace.F1;
        }
        else if(m_id == 19){
            lastFace = ReefFace.F2;

            return ReefFace.F2;
        }
        else if(m_id == 20){
            lastFace = ReefFace.F3;

            return ReefFace.F3;
        }
        else if(m_id == 21){
            lastFace = ReefFace.F4;

            return ReefFace.F4;
        }
        else if(m_id == 22){
            lastFace = ReefFace.F5;

            return ReefFace.F5;
        }
        else if(m_id == 6){
            lastFace = ReefFace.F6;

            return ReefFace.F6;
        }
        else if(m_id == 7){
            lastFace = ReefFace.F1;

            return ReefFace.F1;
        }
        else if(m_id == 8){
            lastFace = ReefFace.F2;

            return ReefFace.F2;
        }
        else if(m_id == 9){
            lastFace = ReefFace.F3;

            return ReefFace.F3;
        }
        else if(m_id == 10){
            lastFace = ReefFace.F4;

            return ReefFace.F4;
        }
        else if(m_id == 11){
            lastFace = ReefFace.F5;

            return ReefFace.F5;
        }
        else if(m_id == 12){
            lastFace = ReefFace.HR;

            return ReefFace.HR;
        }
        else if(m_id == 1){
            lastFace = ReefFace.HR;

            return ReefFace.HR;
        }
        else if(m_id == 2){
            lastFace = ReefFace.HL;

            return ReefFace.HL;
        }
        else if(m_id == 13){
            lastFace = ReefFace.HL;

            return ReefFace.HL;
        }
        else{
            return lastFace;
        }
    }
    else return lastFace;
    }

    public static ReefUtill getReefFacePoint(ReefFace state ){
        switch (state) {

            case F1:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 3.18, 4.2), new Translation2d(fieldLength - 3.18, 3.87), Rotation2d.fromDegrees(180.0) , m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(3.18, 4.2), new Translation2d(3.18, 3.87), Rotation2d.fromDegrees(0.0), m_bool);
                }
            case F2:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 3.97, 5.23), new Translation2d(fieldLength - 3.69 , 5.07), Rotation2d.fromDegrees(240.0), m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(3.97, 5.23), new Translation2d(3.69 , 5.07), Rotation2d.fromDegrees(60.0) , m_bool);
                }

            case F3:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 5.26, 5.07), new Translation2d(fieldLength - 4.98, 5.23), Rotation2d.fromDegrees(300.0), m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(5.26, 5.07), new Translation2d(4.98, 5.23), Rotation2d.fromDegrees(120.0) , m_bool);
                }

            case F4:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 5.77, 3.87), new Translation2d(fieldLength - 5.77, 4.2), Rotation2d.fromDegrees(0.0), m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(5.77, 3.87), new Translation2d(5.77, 4.2), Rotation2d.fromDegrees(180) ,m_bool);
                }

            case F5:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 4.98 , 2.83), new Translation2d(fieldLength - 5.26 , 3), Rotation2d.fromDegrees(120.0), m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(4.98 , 2.83), new Translation2d(5.26 , 3), Rotation2d.fromDegrees(-120.0), m_bool);
                }

            case F6:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 3.69, 3.0), new Translation2d(fieldLength - 3.97, 2.83), Rotation2d.fromDegrees(60.0), m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(3.69, 3.0), new Translation2d(3.97, 2.83), Rotation2d.fromDegrees(-60.0), m_bool);
                }
            case HR:
            if(Robot.s_Alliance == Alliance.Red){//TODO put right pos
                    return new ReefUtill(new Translation2d(fieldLength - 1.1, 1.02), new Translation2d(fieldLength - 3.18, 3.87), Rotation2d.fromDegrees(50.0), m_bool);
             }
             else{
            return new ReefUtill(new Translation2d(3.18, 4.2), new Translation2d(3.18, 3.87), Rotation2d.fromDegrees(50.0), m_bool);
            }
            case HL:
            if(Robot.s_Alliance == Alliance.Red){//TODO put right pos
            return new ReefUtill(new Translation2d(fieldLength - 1.1, 7.04), new Translation2d(fieldLength - 3.18, 3.87), Rotation2d.fromDegrees(310.0), m_bool);
            }
            else{
            return new ReefUtill(new Translation2d(3.18, 4.2), new Translation2d(3.18, 3.87), Rotation2d.fromDegrees(310.0), m_bool);
            }

            default:
                if(Robot.s_Alliance == Alliance.Red){
                    return new ReefUtill(new Translation2d(fieldLength - 0.0, 0.0), new Translation2d(fieldLength - 0.0, 0.0), Rotation2d.fromDegrees(90.0), m_bool);
                }
                else{
                    return new ReefUtill(new Translation2d(0.0, 0.0), new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(90.0), m_bool);
                }
         }
    }
}
