package frc.robot.subsystems;

import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraType;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class VisionSubsystemRobot2025 extends VisionSubsystem {
    
    int regularPipeline = 0;
    int pipelineX2 = 1;

    int[] visionID = { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 };
    //second limelight values
    private double txNote, tyNote;
    private boolean tvNote = false; 

    public VisionSubsystemRobot2025(CameraSettings aprilTagCameraSettings, CameraSettings gamePieceCameraSettings) {
        super(aprilTagCameraSettings, gamePieceCameraSettings);
    }
    @Override
	public void periodic() {
        super.periodic();
        this.orbitCalculation();
    }
    /*
	 * here we are caculate our crop setting we are doing in by using the camera
	 * fov and the limelight values to crop the pic we are doing this because
	 * crop the full picture to maxmize the limelight FPS
	 */
    public void orbitCalculation(){
        double outerLayer = 10.0;

        if(LimelightHelpers.getCurrentPipelineIndex(CameraType.AprilTagCamera.getCameraName()) == pipelineX2){
            outerLayer = outerLayer / 2;
        }

        if (!this.tv) {
            this.cropXMin = -1.5;
            this.cropXMax = 1.5;
            this.cropYMin = -1.5;
            this.cropYMax = 1.5;
        }
        this.crop( this.cropXMin , this.cropXMax , this.cropYMin , this.cropYMax );
    }
    public double getTxNote(){return this.txNote;}
	public double getTyNote(){return this.tyNote;}
	public boolean getTvNote() {return this.tvNote;}
}
