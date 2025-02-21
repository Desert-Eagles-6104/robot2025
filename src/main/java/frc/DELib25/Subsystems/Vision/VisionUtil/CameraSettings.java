package frc.DELib25.Subsystems.Vision.VisionUtil;

public class CameraSettings {
    public double m_forward;
    public double m_Side;
    public double m_up;
    public double m_roll;
    public double m_pitch;
    public double m_yaw;
    public boolean m_is3D;

    public CameraSettings(double forward, double Side, double up, double roll, double pitch,double yaw, boolean is3D){
        m_forward = forward;
        m_Side = Side;
        m_up = up;
        m_roll = roll;
        m_pitch = pitch;
        m_yaw = yaw;
        m_is3D = is3D;
    }
}