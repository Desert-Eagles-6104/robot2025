package frc.DELib25.Subsystems.Vision.VisionUtil;

public enum CameraType {
    AprilTagCamera("limelight-april"),
    GamePieceCamera("limelight");

    final String cameraName;

    CameraType(String cameraName) {
        this.cameraName = cameraName;
    }
    
    public String getCameraName() {
        return cameraName;
    }
}