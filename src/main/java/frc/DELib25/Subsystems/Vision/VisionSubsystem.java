package frc.DELib25.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraType;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers.PoseEstimate;

public abstract class VisionSubsystem extends SubsystemBase {
	
	
	//First AprilTag Limelight
	private CameraSettings aprilTagCameraSettings = null;

	/**
	 * Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
	 */
	private double tx = 0;

	/**
	 * Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
	 */
	private double ty = 0;
	private double lastTy = 0;
	private double lastTx = 0;

	/**
	 * Whether the limelight has any valid targets (0 or 1)
	 */
	protected boolean tv = false;
	private PoseEstimate estimatedRobotPose = new PoseEstimate(); 
	private double currentID = 0;
    
    protected double cropXMin = -1;
    protected double cropXMax = 1;
    protected double cropYMin = -1;
    protected double cropYMax = 1;
    
    protected double xFOV = 62.5;
    protected double yFOV = 48.9;
    
    //*create a new VisionSubsystem constructor to apply the subsystem's properties */
    public VisionSubsystem(CameraSettings aprilTagCameraSettings, CameraSettings gamePieceCameraSettings) {
      this.aprilTagCameraSettings = aprilTagCameraSettings;
      if(aprilTagCameraSettings != null){
			LimelightHelpers.setCameraPose_RobotSpace(
				CameraType.AprilTagCamera.getCameraName(),
				aprilTagCameraSettings.m_forward,
				aprilTagCameraSettings.m_Side,
				aprilTagCameraSettings.m_up,
				aprilTagCameraSettings.m_roll,
				aprilTagCameraSettings.m_pitch,
				aprilTagCameraSettings.m_yaw
			);
      	}
      LimelightHelpers.setPipelineIndex(CameraType.AprilTagCamera.getCameraName(), 0);
    }
  
    @Override
	public void periodic() {
		updateVisionData();

		SmartDashboard.putString("limeName", CameraType.AprilTagCamera.getCameraName());

		SmartDashboard.putNumber("TX", getTx());
		SmartDashboard.putNumber("TY", getTy());
		SmartDashboard.putBoolean("TV", getTv());
	}

	protected void updateVisionData() {
		this.tv = LimelightHelpers.getTV(CameraType.AprilTagCamera.getCameraName());
		if (this.tv) {
			this.tx = LimelightHelpers.getTX(CameraType.AprilTagCamera.getCameraName(), this.lastTx);
			this.ty = LimelightHelpers.getTY(CameraType.AprilTagCamera.getCameraName(), this.lastTy);
			this.currentID = LimelightHelpers.getFiducialID(CameraType.AprilTagCamera.getCameraName());
			this.estimatedRobotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraType.AprilTagCamera.getCameraName());
			this.lastTy = this.ty;
			this.lastTx = this.tx;
		}
	}
	
	
	
	/**
	 * Returns the estimated robot pose based on the AprilTag camera's data.
	 * If no pose is estimated, it returns a default PoseEstimate with a zero Pose2d.
	 * @return
	 */
	public PoseEstimate getEstimatedRobotPose() {
		if (this.estimatedRobotPose == null) {
			PoseEstimate poseEstimate = new PoseEstimate();
			poseEstimate.pose = new Pose2d();
			return poseEstimate;
		}
		return this.estimatedRobotPose;
	}
    
    
    public double getTx(){return this.tx;}
    
	public double getTy() {
		return this.ty + this.aprilTagCameraSettings.m_pitch;
	}
	
	public boolean getTv(){return this.tv;}
	
	
	/**
	 * Crops the camera image to the specified window.
	 * The crop values are in the range of -1.0 to 1.0, where -1.0 is the left/top edge and 1.0 is the right/bottom edge.
	 * @param cropXMin
	 * @param cropXMax
	 * @param cropYMin
	 * @param cropYMax
	 */
    public void crop(double cropXMin, double cropXMax, double cropYMin, double cropYMax){
      LimelightHelpers.setCropWindow(CameraType.AprilTagCamera.getCameraName(), cropXMin, cropXMax, cropYMin, cropYMax);
    }
    
    /**
     * @return Total vision latency (photons -> robot) in seconds
     */
    public double getTotalLatency() {
      double miliToSec = 0.001;
      return LimelightHelpers.getLatency_Pipeline(CameraType.AprilTagCamera.getCameraName()) + LimelightHelpers.getLatency_Capture(CameraType.AprilTagCamera.getCameraName()) * miliToSec;
    }
  
	public double getCurrentID() {
		return this.currentID;
	}
	
}

