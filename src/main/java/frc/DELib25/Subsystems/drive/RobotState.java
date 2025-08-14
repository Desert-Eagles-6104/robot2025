package frc.DELib25.Subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
   private static Pose2d robotToFieldFromSwerveDriveOdometry = new Pose2d();

    public static void setRobotToFieldFromSwerveDriveOdometry(Pose2d pose) {
        robotToFieldFromSwerveDriveOdometry = pose;
    }
    
    public static Pose2d getRobotToFieldFromSwerveDriveOdometry() {
        return robotToFieldFromSwerveDriveOdometry;
    }
}
