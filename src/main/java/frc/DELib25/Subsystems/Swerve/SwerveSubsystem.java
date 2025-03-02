// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.CSV.CSVReader;
import frc.DELib25.CSV.CSVWriter;
import frc.DELib25.Intepulation.InterpolatingDouble;
import frc.DELib25.Intepulation.InterpolatingTreeMap;
import frc.DELib25.Sensors.Pigeon;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem swerve = null;

  private SwerveConstants m_swerveConstants;

  private CSVReader m_reader;
  private CSVWriter m_writer;

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  private SwerveModule[] m_swerveModules = new SwerveModule[4];
  
  private Field2d m_field = new Field2d();

  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_odometry;
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> m_pastPoses;

  private Pigeon m_gyro;

  /** Creates a new SwerveSubsystem */
  private SwerveSubsystem(SwerveConstants swerveConstants) {
    m_swerveConstants = swerveConstants;
    
    Pigeon.CreateInstance(44);
    m_gyro = Pigeon.getInstance();

    m_frontLeft = new SwerveModule(swerveConstants.FL, swerveConstants);
    m_frontRight = new SwerveModule(swerveConstants.FR, swerveConstants);
    m_backLeft = new SwerveModule(swerveConstants.BL, swerveConstants);
    m_backRight = new SwerveModule(swerveConstants.BR, swerveConstants);
    m_swerveModules[0] = m_frontLeft;
    m_swerveModules[1] = m_frontRight;
    m_swerveModules[2] = m_backLeft;
    m_swerveModules[3] = m_backRight;

    SmartDashboard.putData("Field", m_field);

    m_kinematics = new SwerveDriveKinematics(swerveConstants.frontLeftPos, swerveConstants.frontRightPos, swerveConstants.backLeftPos, swerveConstants.backRightPos);
    m_odometry = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(0), getModulesPositions(), new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.3, 0.3, 9999999));
    
    // readAngleOffsets();

    // try {
    //   m_writer = new CSVWriter(swerveConstants.filepath);
    //   m_reader = new CSVReader(swerveConstants.filepath);
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }

    int k_maxPoseHistorySize = 51;
    m_pastPoses = new InterpolatingTreeMap<>(k_maxPoseHistorySize);
  }

  public void drive(ChassisSpeeds chassisSpeeds , boolean openLoop , boolean fieldRelative, Translation2d centerOfRtation){
    Rotation2d heading = (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) ? getHeading().plus(Rotation2d.fromDegrees(180)) : getHeading();
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] states = fieldRelative ? m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, heading), centerOfRtation) : m_kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRtation);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConstants.maxSpeed);
    setModulesStates(states , openLoop, true);
  }

  public void setModulesStates(SwerveModuleState[] states , boolean isOpenLoop, boolean avoidJittering){
    for (int i = 0; i < states.length; i++) {
      m_swerveModules[i].setState(states[i], isOpenLoop, avoidJittering);
    }
  }

  public void setModulesNetrualMode(NeutralModeValue neutralMode){
    for (int i = 0; i < m_swerveModules.length; i++){
      m_swerveModules[i].setSteeringNeturalMode(neutralMode);
    }
  }

  public Rotation2d getHeading(){
    return getPose().getRotation();
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp){
    m_odometry.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(0.7, 0.7, 9999999));
  }

  public SwerveDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public Pose2d getInterpolatedPose(double latencySeconds){
    double timestamp = Timer.getFPGATimestamp() - latencySeconds;
    return m_pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public SwerveModulePosition[] getModulesPositions(){ 
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = m_swerveModules[i].getModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < states.length; i++) {
      states[i] = m_swerveModules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getRobotRelativeVelocity(){
    return m_kinematics.toChassisSpeeds(getStates());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule m : m_swerveModules) {
      m.refreshAllSignals();
    }
    m_gyro.getYawStatusSignal().refresh();
    Pose2d currentPose = m_odometry.update(m_gyro.getYaw(), getModulesPositions());
    m_pastPoses.put(new InterpolatingDouble(Timer.getFPGATimestamp()), currentPose);
    SmartDashboard.putNumber("RobotHeading", getHeading().getDegrees());
    m_odometry.update(m_gyro.getYaw() ,getModulesPositions());
    m_field.setRobotPose(m_odometry.getEstimatedPosition());
  }

  public void zeroHeading(){
    Rotation2d heading = (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) ? Rotation2d.fromDegrees(180) : new Rotation2d();
    m_odometry.resetPosition(m_gyro.getYaw(), getModulesPositions(), new Pose2d(getPose().getTranslation(), heading));
    m_gyro.setYaw(heading.getDegrees());
  }

  public void resetToAbsolute(){
    for (int i = 0; i < m_swerveModules.length; i++){
      m_swerveModules[i].resetToAbsolute();
    }

    
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getYaw(), getModulesPositions(), pose);
  }

  public void disableModules(){
    for(int i = 0; i < m_swerveModules.length; i++){
      m_swerveModules[i].DisableMotors();
    }
  }

  public void runSwerveCharacterization(Voltage volts){
    for(int i = 0; i < 4; i++){
      m_swerveModules[i].runCharacterization(volts.in(Volts));
   }
  }

  // public void readAngleOffsets(){
  //   // double[][] angleOffsets = m_reader.readAsDouble(1);
  //   double [] angleOffsets = new double[4];
  //   for(int i = 0; i < angleOffsets.length; i++){
  //     angleOffsets[i]= Constants.Swerve.swerveConstants.angleOffset[i];
  //   }

  //   for(int i = 0; i < angleOffsets.length; i++){
  //     m_swerveModules[i].setAngleOffset(Rotation2d.fromRotations(angleOffsets[i]));
  //   }
  // }

  // public boolean updateAngleOffsets(){
  //   double[][] angleOffsets = new double[4][1];
  //   for(int i = 0; i < angleOffsets.length; i++){
  //     Rotation2d angleOffset = m_swerveModules[i].getAbsAngle();
  //     angleOffsets[i][0] = angleOffset.getRotations();
  //     m_swerveModules[i].setAngleOffset(angleOffset);
  //   }
  //   double[] smartdashboardArray = {angleOffsets[0][0], angleOffsets[1][0] ,angleOffsets[2][0] ,angleOffsets[3][0]};
  //   SmartDashboard.putNumberArray("offsets", smartdashboardArray);
  //   m_writer.writeCSVFile(angleOffsets);
  //   return true;
  // }

  public static SwerveSubsystem createInstance(SwerveConstants swerveConstants){
    if(swerve == null){
      swerve = new SwerveSubsystem(swerveConstants);
    }
    return swerve;
  } 

  public static SwerveSubsystem getInstance(){
    if(swerve != null){
      return swerve;  
    }
    return null;
  } 
}
