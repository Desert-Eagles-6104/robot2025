package frc.DELib25.Subsystems.Vision.VisionUtil;

public class CameraSettings {
    public double forward;
    public double side;
    public double up;
    public double roll;
    public double pitch;
    public double yaw;
    public boolean is3D;

    public CameraSettings(double forward, double Side, double up, double roll, double pitch,double yaw, boolean is3D){
        this.forward = forward;
        this.side = Side;
        this.up = up;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
        this.is3D = is3D;
    }
}