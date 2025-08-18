package frc.DELib25.Subsystems.Drive.SwerveUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.BooleanUtil.LatchedBolean;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Util.ProjectConstants;

public class HeadingController  {

    private PIDController m_pidController;
    private PIDContainer m_pidContainerStabalize;
    private PIDContainer m_pidContainerSnap;
    private PIDContainer m_pidContainerVision;
    private PIDContainer m_pidContainerVisionLowError;

    private PIDContainer m_currentPIDContainer = new PIDContainer(0, 0, 0, "");


    private Mode mode = Mode.TeleopWithHeadingController;

    private Rotation2d m_lastHeading;
    private boolean m_shouldSaveHeading = true;
    private LatchedBolean m_useVisionLatch;
    private boolean firstRun = true;

   /**
    *  maintains the set direction by the joystick within a 3 degree error of the joystick
    * @param kp p value
    * @param ki i value
    * @param kd d value
    */
    public HeadingController (double kp , double ki , double kd){
        m_pidController = new PIDController(kp, ki, kd);
        m_pidController.enableContinuousInput(-180, 180);
        m_pidController.setIntegratorRange(-1*(9.9*Math.E+30), (9.9*Math.E+30));
        m_useVisionLatch = new LatchedBolean();
    }

    /**
    *  maintains the set direction by the joystick within a 3 degree error of the joystick
    * @param kp p value
    * @param ki i value
    * @param kd d value
    */
    public HeadingController (PIDContainer pidSettings){
        m_pidController = new PIDController(pidSettings.kP, pidSettings.kI, pidSettings.kD);
        m_pidController.enableContinuousInput(-180, 180);
        m_pidController.setIntegratorRange(-1*(9.9*Math.E+30), (9.9*Math.E+30));
        m_useVisionLatch = new LatchedBolean();
    }

        /**
    *  maintains the set direction by the joystick within a 3 degree error of the joystick
    * @param kp p value
    * @param ki i value
    * @param kd d value
    */
    public HeadingController (PIDContainer stabalize, PIDContainer snap, PIDContainer vision, PIDContainer visionLowError){
        m_pidController = new PIDController(stabalize.kP, stabalize.kI, stabalize.kD);
        m_pidController.enableContinuousInput(-180, 180);
        m_pidController.setIntegratorRange(-1*(9.9*Math.E+30), (9.9*Math.E+30));
        m_useVisionLatch = new LatchedBolean();

        m_pidContainerStabalize = stabalize;
        m_pidContainerSnap = snap;
        m_pidContainerVision = vision;
        m_pidContainerVisionLowError = visionLowError;
    }
    
    /**
     * holds the current heading of the swerve and sets it to the pid controller
     * @param currentHeading current heading of the robot
     * @return a setpoint to the pid controller
     */
    public double update(Rotation2d currentHeading){
        return clamp(m_pidController.calculate(currentHeading.getDegrees()),-ProjectConstants.Swerve.swerveConstants.maxAngularVelocity,ProjectConstants.Swerve.swerveConstants.maxAngularVelocity);
    }

    /**
     * calculates the heading of the robot
     * @param shouldRun a boolean to decide whether or not to use the headingController function
     * @param chassisSpeeds x,y and omega speeds 
     * @return new values for the modules to use 
     */
    public ChassisSpeeds calculateOmegaSpeed(boolean shouldRun ,boolean isSwerveReset, boolean useVision ,ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Rotation2d visionHeading){
      m_useVisionLatch.update(useVision);
      if(shouldRun && !isSwerveReset){
        if(firstRun){
          setSetpoint(robotHeading);
          firstRun = false;
        }
        if(!m_useVisionLatch.get()){
          if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.05){ // was 0.15
            if(m_shouldSaveHeading){
              m_lastHeading = robotHeading;
              setSetpoint(m_lastHeading);
              m_shouldSaveHeading = false;
            }
            chassisSpeeds.omegaRadiansPerSecond = update(robotHeading);
            return chassisSpeeds;
          }
          else{
            m_shouldSaveHeading= true;
          }
        }
        else if(m_useVisionLatch.get()){
          if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.05){
            m_useVisionLatch.reset();
          }
          chassisSpeeds.omegaRadiansPerSecond = update(visionHeading);
          return chassisSpeeds;
        }
      }
      else if(isSwerveReset){
        setSetpoint(DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? Rotation2d.fromDegrees(180):Rotation2d.fromDegrees(0)); 
      }
      return chassisSpeeds;
  }
    
  /**
   * sets a setpoint for the pid controller
   * @param setpoint
   */
  public void setSetpoint(Rotation2d setpoint){
      if(setpoint.getDegrees() == m_pidController.getSetpoint()) return;
      m_pidController.reset();
      m_pidController.setSetpoint(setpoint.getDegrees());
  }

  public ChassisSpeeds calculateOmegaSpeed2(boolean shouldRun, boolean isSwerveReset, boolean useVision ,ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Rotation2d visionHeading, ChassisSpeeds robotRelativeVelocity){
    m_useVisionLatch.update(useVision);
    if(firstRun){
          setSetpoint(robotHeading);
          firstRun = false;
    }

    smartChangePIDController(robotRelativeVelocity);

    if(isSwerveReset) mode = Mode.ResetHeading;
    else if(m_useVisionLatch.get() && shouldRun) mode = Mode.TeleopWithVision;
    else if(shouldRun) mode = Mode.TeleopWithHeadingController;
    else mode = Mode.TeleopWithoutHeadingController;
    
    SmartDashboard.putString("HeadingMode", mode.toString());
    SmartDashboard.putString("currentHeadingType", m_currentPIDContainer.headingType);

    switch (mode) {
      case TeleopWithHeadingController: 
        //BASE        
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.05){
          if(m_shouldSaveHeading){
            m_lastHeading = robotHeading;
            setSetpoint(m_lastHeading);
            m_shouldSaveHeading = false;
          }
          chassisSpeeds.omegaRadiansPerSecond = update(robotHeading);
          return chassisSpeeds;
        }
        else{
          m_shouldSaveHeading = true;
        }
        return chassisSpeeds;
        //BASE

      case TeleopWithoutHeadingController:
        return chassisSpeeds;

      case TeleopWithVision:
        //BASE
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.4){
          m_useVisionLatch.reset();
        }
        chassisSpeeds.omegaRadiansPerSecond = update(visionHeading);
        return chassisSpeeds;
        //BASE

      case ResetHeading:
        //BASE
        setSetpoint(DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? Rotation2d.fromDegrees(180):Rotation2d.fromDegrees(0)); 
        return chassisSpeeds;
        //BASE

      default:
      return chassisSpeeds;
    }  
  }
  
  public void smartChangePIDController(ChassisSpeeds robotRelativeVelocity){
    double velocity = new Translation2d(robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond).getNorm();
    if(m_useVisionLatch.get()){
      // vision
      if(m_pidController.getError() < 1){
        setPIDSettings(m_pidContainerVisionLowError);
      }
      else{
        setPIDSettings(m_pidContainerVision);
      }
    }
    else if(velocity > ProjectConstants.Swerve.swerveConstants.maxSpeed * 0.3){
      // stabalize
      setPIDSettings(m_pidContainerStabalize);
    }
    else{
      // snap
      setPIDSettings(m_pidContainerSnap);
    }
  }

  /**
  @param value clamped value
  @param min min value
  @param max max value
  @return sets a range for the value if its between the max and min points
  */
  private double clamp(double value, double min, double max) {
  return Math.max(min, Math.min(max, value));
  }

  private void setPIDSettings(PIDContainer pidSettings){
    if(!m_currentPIDContainer.headingType.equals(pidSettings.headingType)){
      m_pidController.setPID(pidSettings.kP, pidSettings.kI, pidSettings.kD);
      m_currentPIDContainer.headingType = pidSettings.headingType;
    }
  }

  public enum Mode {
  TeleopWithHeadingController,
  TeleopWithoutHeadingController,
  TeleopWithVision,
  ResetHeading
  }
}