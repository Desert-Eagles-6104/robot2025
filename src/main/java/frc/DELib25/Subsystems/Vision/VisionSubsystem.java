// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  
  //First AprilTag Limelight
  private static CameraSettings m_aprilTagCameraSettings = null;
  private static double m_tx = 0; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  private static double m_ty = 0; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  private double m_lastTy = 0;
  private double m_lastTx = 0;
  private static boolean m_tv = false; //Whether the limelight has any valid targets (0 or 1)
  private static LimelightHelpers.PoseEstimate m_estimatedRobotPose = new LimelightHelpers.PoseEstimate(); 
  private double m_currentID = 0;
  
  private double cropXMin = -1;
  private double cropXMax = 1;
  private double cropYMin = -1;
  private double cropYMax = 1;
  
  double xFOV = 62.5;
  double yFOV = 48.9;
  
  int regularPipeline = 0;
  int pipelineX2 = 1;

  int[] visionID = {7};

  int[] localizationVisionID = {7,8,6};

  //second limelight values
  private static double m_TxNote = 0;
  private static double m_TyNote = 0; 
  private double m_LastTyNote = 0;
  private double m_LastTxNote = 0;
  private static boolean m_TvNote = false; 

  //*create a new VisionSubsystem constructor to apply the subsystem's properties */
  public VisionSubsystem(CameraSettings aprilTagCameraSettings, CameraSettings gamePieceCameraSettings) {
    m_aprilTagCameraSettings = aprilTagCameraSettings;
    if(aprilTagCameraSettings != null){
      LimelightHelpers.setCameraPose_RobotSpace(CameraType.AprilTagCamera.getCameraName(), aprilTagCameraSettings.m_forward, aprilTagCameraSettings.m_Side, aprilTagCameraSettings.m_up, aprilTagCameraSettings.m_roll, aprilTagCameraSettings.m_pitch, aprilTagCameraSettings.m_yaw);
    }
  }

  @Override
  public void periodic() {
    m_tv = LimelightHelpers.getTV(CameraType.AprilTagCamera.getCameraName());
    if(m_tv){
      m_tx = LimelightHelpers.getTX(CameraType.AprilTagCamera.getCameraName(), m_lastTx);
      m_ty = LimelightHelpers.getTY(CameraType.AprilTagCamera.getCameraName(), m_lastTy);
      m_currentID = LimelightHelpers.getFiducialID(CameraType.AprilTagCamera.getCameraName());
      m_estimatedRobotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraType.AprilTagCamera.getCameraName());
      m_lastTy = m_ty;
      m_lastTx = m_tx;
    }

   // m_TvNote = LimelightHelpers.getTV(CameraType.GamePieceCamera.getCameraName());
   // if(m_TvNote){
   //   m_TxNote = LimelightHelpers.getTX(CameraType.GamePieceCamera.getCameraName(), m_LastTxNote);
   //   m_TyNote = LimelightHelpers.getTY(CameraType.GamePieceCamera.getCameraName(), m_LastTyNote);
   //   m_LastTyNote = m_TyNote;
   //   m_LastTxNote = m_TxNote;
   // }

    //bounding april tag
    orbitCalculation();
    //limelight values
    SmartDashboard.putNumber("TX", getTx());
    SmartDashboard.putNumber("TY", getTy());
    SmartDashboard.putBoolean("TV", getTv());
  }

  public static LimelightHelpers.PoseEstimate getEstimatedRobotPose(){
    if(m_estimatedRobotPose == null){
      PoseEstimate poseEstimate = new PoseEstimate();
      poseEstimate.pose = new Pose2d();
      return poseEstimate;
    }
    return m_estimatedRobotPose;
  }

  public static double getID(){
    return LimelightHelpers.getFiducialID(CameraType.AprilTagCamera.getCameraName());
  }
  
  public static double getTx(){
    return m_tx;
  }
  
  public static double getTy(){
    return m_ty + m_aprilTagCameraSettings.m_pitch;
  }

  public static boolean getTv(){
    return m_tv;
  }

  public static double getTxNote(){
    return m_TxNote;
  }
  
  public static double getTyNote(){
    return m_TyNote;
  }

  public static boolean getTvNote(){
    return m_TvNote;
  }

  public void crop(double cropXMin, double cropXMax, double cropYMin, double cropYMax){
    LimelightHelpers.setCropWindow(CameraType.AprilTagCamera.getCameraName(), cropXMin, cropXMax, cropYMin, cropYMax);
  }
  
  /**   
   * @return Total vision latency (photons -> robot) in seconds
   */
  public static double getTotalLatency() {
    double miliToSec = 0.001;
    return LimelightHelpers.getLatency_Pipeline(CameraType.AprilTagCamera.getCameraName()) + LimelightHelpers.getLatency_Capture(CameraType.AprilTagCamera.getCameraName()) * miliToSec;
  }

    /**   
   * @return Total vision latency (photons -> robot) in seconds
   */
  public static double getTotalLatencyNote() {
    double miliToSec = 0.001;
    return LimelightHelpers.getLatency_Pipeline(CameraType.GamePieceCamera.getCameraName()) + LimelightHelpers.getLatency_Capture(CameraType.GamePieceCamera.getCameraName()) * miliToSec;
  }

  public double getCurrentID(){
    return m_currentID;
  }

  /*
   * here we are caculate our crop setting we are doing in by using the camera fov and the limelight values to crop the pic
   * we are doing this because crop the full picture to maxmize the limelight FPS
   */
  public void orbitCalculation(){
    double precentY = 0.5;
    double precentX = 0.0;
    double outerLayer = 10.0;
    double _xFOV = xFOV;
    double _yFOV = yFOV;

    if(LimelightHelpers.getCurrentPipelineIndex(CameraType.AprilTagCamera.getCameraName()) == pipelineX2){
      _xFOV = xFOV / 2.0;
      _yFOV = yFOV / 2.0;
      outerLayer = outerLayer / 2;
    }

    if(m_tv){
      // cropXMin = (m_tx - outerLayer) / (precentX * _xFOV);
      // cropXMax = (m_tx + outerLayer) / (precentX * _xFOV);
      cropYMin = (m_ty - outerLayer) / (precentY * _yFOV);
      cropYMax = (m_ty + outerLayer) / (precentY * _yFOV);
    }
    else{
      cropXMin = -1.0;
      cropXMax = 1.0;
      cropYMin = -1.0;
      cropYMax = 1.0;
    }
    crop( cropXMin , cropXMax , cropYMin , cropYMax );
  }

  public void changePiplne(int pipeline){
    LimelightHelpers.setPipelineIndex(CameraType.AprilTagCamera.getCameraName(), pipeline);
  }

  public enum CameraType{
    AprilTagCamera("limelight"),
    GamePieceCamera("limelight-notes");

    final String m_cameraName;

    CameraType(String cameraName){
      m_cameraName = cameraName;
    }

    public String getCameraName() {
        return m_cameraName;
    }
  }
}

