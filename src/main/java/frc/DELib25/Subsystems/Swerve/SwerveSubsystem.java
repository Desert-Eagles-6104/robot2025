// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.CSV.CSVReader;
import frc.DELib25.CSV.CSVWriter;
import frc.DELib25.Sensors.Pigeon;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem swerve = null;

  private SwerveConstants swerveConstants;
  // TODO: Implement CSVReader and CSVWriter (but Preferably we move to json)
  private CSVReader reader;
  private CSVWriter writer;

  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;
  private SwerveModule[] swerveModules = new SwerveModule[4];
  

  private SwerveDriveKinematics kinematics;
  

  private Pigeon gyro;
  /** Creates a new SwerveSubsystem */
  private SwerveSubsystem(SwerveConstants swerveConstants) {
    this.swerveConstants = swerveConstants;
    
    this.gyro = new Pigeon(44);

    this.frontLeft = new SwerveModule(swerveConstants.FL, swerveConstants);
    this.frontRight = new SwerveModule(swerveConstants.FR, swerveConstants);
    this.backLeft = new SwerveModule(swerveConstants.BL, swerveConstants);
    this.backRight = new SwerveModule(swerveConstants.BR, swerveConstants);
    this.swerveModules[0] = this.frontLeft;
    this.swerveModules[1] = this.frontRight;
    this.swerveModules[2] = this.backLeft;
    this.swerveModules[3] = this.backRight;

    this.kinematics = new SwerveDriveKinematics(swerveConstants.frontLeftPos, swerveConstants.frontRightPos, swerveConstants.backLeftPos, swerveConstants.backRightPos);
    // readAngleOffsets();

    // try {
    //   m_writer = new CSVWriter(swerveConstants.filepath);
    //   m_reader = new CSVReader(swerveConstants.filepath);
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }
  
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean openLoop, boolean fieldRelative,Translation2d centerOfRtation) {
    boolean isRedAlliance = DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    Rotation2d heading = isRedAlliance ? this.gyro.getYaw().plus(Rotation2d.fromDegrees(180)) : this.gyro.getYaw();
    
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] states = fieldRelative
        ? this.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, heading),centerOfRtation)
        : this.kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRtation);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(states, this.swerveConstants.maxSpeed);
    setModulesStates(states , openLoop, true);
  }

  public void setModulesStates(SwerveModuleState[] states , boolean isOpenLoop, boolean avoidJittering){
    for (int i = 0; i < states.length; i++) {
      this.swerveModules[i].setState(states[i], isOpenLoop, avoidJittering);
    }
  }

  public void setModulesNetrualMode(NeutralModeValue neutralMode){
    for (int i = 0; i < this.swerveModules.length; i++){
      this.swerveModules[i].setSteeringNeturalMode(neutralMode);
    }
  }

  public Pigeon getGyro(){ return this.gyro; }
  public SwerveDriveKinematics getKinematics(){ return this.kinematics; }


  public SwerveModulePosition[] getModulesPositions(){ 
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = this.swerveModules[i].getModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < states.length; i++) {
      states[i] = this.swerveModules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getRobotRelativeVelocity(){
    return this.kinematics.toChassisSpeeds(getStates());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule module : this.swerveModules) {
      module.refreshAllSignals();
    }
  }
  /** TODO: rewrite this function properly
  public void zeroHeading(){
    Rotation2d heading = (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) ? Rotation2d.fromDegrees(180) : new Rotation2d();
    this.poseEstimator.resetOdometry(new Pose2d(this.poseEstimator.getPose().getTranslation(), heading));
    this.gyro.setYaw(heading.getDegrees());
  }*/

  public void resetToAbsolute(){
    for (int i = 0; i < this.swerveModules.length; i++){
      this.swerveModules[i].resetToAbsolute();
    }

    
  }


  public void disableModules(){
    for(int i = 0; i < this.swerveModules.length; i++){
      this.swerveModules[i].DisableMotors();
    }
  }

  public void runSwerveCharacterization(Voltage volts){
    for(int i = 0; i < 4; i++){
      this.swerveModules[i].runCharacterization(volts.in(Volts));
   }
  }

  // public void readAngleOffsets(){
  //   // double[][] angleOffsets = m_reader.readAsDouble(1);
  //   double [] angleOffsets = new double[4];
  //   for(int i = 0; i < angleOffsets.length; i++){
  //     angleOffsets[i]= Constants.Swerve.swerveConstants.angleOffset[i];
  //   }

  //   for(int i = 0; i < angleOffsets.length; i++){
  //     this.swerveModules[i].setAngleOffset(Rotation2d.fromRotations(angleOffsets[i]));
  //   }
  // }

  // public boolean updateAngleOffsets(){
  //   double[][] angleOffsets = new double[4][1];
  //   for(int i = 0; i < angleOffsets.length; i++){
  //     Rotation2d angleOffset = this.swerveModules[i].getAbsAngle();
  //     angleOffsets[i][0] = angleOffset.getRotations();
  //     this.swerveModules[i].setAngleOffset(angleOffset);
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
