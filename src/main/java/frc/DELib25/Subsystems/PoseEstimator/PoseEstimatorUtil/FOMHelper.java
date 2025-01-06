package frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class FOMHelper {
    
    public static double cameraFOMBasedOnRobotVelocity(double currentVelocity , double maxVelocity , double standingInPlaceFOM , double fullSpeedFOM){
        double VelocityPrecentage = Math.abs(currentVelocity) / maxVelocity;
        double cameraFOM = (fullSpeedFOM - standingInPlaceFOM) * VelocityPrecentage;
        return cameraFOM;
    }

    public static double skewOverTime(double meterPerDegrees, double deltaSwerveRotation, double currentOdometryFOM){
        return currentOdometryFOM + (deltaSwerveRotation * meterPerDegrees);
    }

    public static boolean isCollision(double robotAccelaration, double gyroAcclaration, double maxG){
        // BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
        // LinearFilter xLinearFilter = LinearFilter.movingAverage(10);
        // xLinearFilter.calculate(accelerometer.getX());
        return Math.abs(robotAccelaration - gyroAcclaration) > maxG;
    }

    /**
     * the skidding module is the one with the highest velocity
     * gets the skidding ratio from the latest , that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the  ratio between the maximum and minimum magnitude of the "translational" part of the speed of the modules
     * 
     * @param swerveStatesMeasured the swerve states measured from the modules
     * @param swerveDriveKinematics the kinematics
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     * */
    public static double getSkiddingRatio(SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
        final double angularVelocityOmegaMeasured = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
        final double[] swerveStatesTranslationalPartMagnitudes = new double[swerveStatesMeasured.length];

        for (int i =0; i < swerveStatesMeasured.length; i++) {
            final Translation2d swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
                    swerveStatesRotationalPartAsVector = convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
                    swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
        for (double translationalSpeed:swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

    private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
        return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }
}