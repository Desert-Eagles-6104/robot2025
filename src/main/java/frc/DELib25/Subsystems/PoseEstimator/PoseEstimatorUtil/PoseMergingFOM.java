// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PoseMergingFOM {

    private double m_cameraFOM = Double.POSITIVE_INFINITY;
    private double m_odometryFOM = Double.POSITIVE_INFINITY;

    public PoseMergingFOM(){
    }

    public Pose2d FOMCalculation(Pose2d odometry, Pose2d camera){
        double x = FOMValueCalculation(odometry.getX(), camera.getX(), m_odometryFOM, m_cameraFOM);
        double y = FOMValueCalculation(odometry.getY(), camera.getY(), m_odometryFOM, m_cameraFOM);
        double omega = FOMValueCalculation(odometry.getRotation().getDegrees(), camera.getRotation().getDegrees(), m_odometryFOM, m_cameraFOM);
        return new Pose2d(new Translation2d(x,y), Rotation2d.fromDegrees(omega));
    }

    private double FOMValueCalculation(double valueOne, double valueTwo, double FOMOne, double FOMTwo){
        return ((valueOne / FOMOne) + (valueTwo/ FOMTwo)) / ((1/FOMOne)+(1/FOMTwo));
    }

    public void updateCameraFOM(double cameraFOM){
        m_cameraFOM = cameraFOM;
    }

    public void updateOdometryFOM(double odometryFOM){
        m_odometryFOM = odometryFOM;
    }
}
