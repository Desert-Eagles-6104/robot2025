package frc.DELib25.Subsystems.Vision;

import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraType;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class VisionSubsystemRobot2025 extends VisionSubsystem {
    
    int regularPipeline = 0;
    int pipelineX2 = 1;

    int[] visionID = { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 };


    public VisionSubsystemRobot2025(CameraSettings aprilTagCameraSettings, CameraSettings gamePieceCameraSettings) {
        super(aprilTagCameraSettings, gamePieceCameraSettings);
    }

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

        if (this.tv) {
            
        }
        else{
            this.cropXMin = -1.5;
            this.cropXMax = 1.5;
            this.cropYMin = -1.5;
            this.cropYMax = 1.5;
        }
        this.crop( this.cropXMin , this.cropXMax , this.cropYMin , this.cropYMax );
    }
}
