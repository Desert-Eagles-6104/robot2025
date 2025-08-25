package frc.DELib25.Subsystems.Drive;
/** High-level operating modes for the swerve subsystem. */
public enum DriveState {
    /** System identification/characterization routines (SysId). Robot ignores driver input. */
    SYS_ID,
  
    /** Normal teleop driving from joysticks (field or robot relative per your code). */
    TELEOP_DRIVE,
  
    /** Following a Choreo trajectory (pose & velocity targets come from the path). */
    CHOREO_PATH,
  
    /** Locks robot heading to a target angle while allowing X/Y translation. */
    ROTATION_LOCK,
  
    /** Drives to a specific field pose (PID to a pose target). */
    DRIVE_TO_POINT,
}