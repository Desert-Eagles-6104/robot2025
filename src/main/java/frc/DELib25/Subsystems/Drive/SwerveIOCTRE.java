package frc.DELib25.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.DELib25.Util.FieldUtil;
import frc.DELib25.Util.ProjectConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class SwerveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveIO {

    HashMap<String, BaseStatusSignal> frontLeftSignals = new HashMap<>();
    HashMap<String, BaseStatusSignal> frontRightSignals = new HashMap<>();
    HashMap<String, BaseStatusSignal> backLeftSignals = new HashMap<>();
    HashMap<String, BaseStatusSignal> backRightSignals = new HashMap<>();

    Map<Integer, HashMap<String, BaseStatusSignal>> signalsMap = new HashMap<>();

    @SafeVarargs
    public SwerveIOCTRE(
            SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
        ) {
            
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);
        this.resetRotation(FieldUtil.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);

        signalsMap.put(0, frontLeftSignals);
        signalsMap.put(1, frontRightSignals);
        signalsMap.put(2, backLeftSignals);
        signalsMap.put(3, backRightSignals);

        for (int i = 0; i < 4; i++) {
            var driveMotor = this.getModule(i).getDriveMotor();
            var steerMotor = this.getModule(i).getSteerMotor();

            var moduleMap = signalsMap.get(i);

            moduleMap.put("driveSupplyCurrentAmps", driveMotor.getSupplyCurrent());
            moduleMap.put("driveStatorCurrentAmps", driveMotor.getStatorCurrent());
            moduleMap.put("driveAppliedVolts", driveMotor.getMotorVoltage());
            moduleMap.put("driveTemperature", driveMotor.getDeviceTemp());

            moduleMap.put("steerSupplyCurrentAmps", steerMotor.getSupplyCurrent());
            moduleMap.put("steerStatorCurrentAmps", steerMotor.getStatorCurrent());
            moduleMap.put("steerAppliedVolts", steerMotor.getMotorVoltage());
            moduleMap.put("steerTemperature", steerMotor.getDeviceTemp());
        }
    }

    @Override
    public void registerTelemetryFunction(SwerveIOInputs inputs) {
        this.registerTelemetry(state -> {
            SwerveDriveState modifiedState = (SwerveDriveState) state;
            modifiedState.Speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    ((SwerveDriveState) state).Speeds, ((SwerveDriveState) state).Pose.getRotation());
            inputs.logState(modifiedState);
        });
    }

    @Override
    public void updateSwerveInputs(SwerveIOInputs inputs) {
        var state = this.getStateCopy();
        state.Speeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        inputs.logState(state);
    }

    @Override
    public void setSwerveState(SwerveRequest request) {
        this.setControl(request);
    }

    @Override
    public void resetRotation() {
        this.resetRotation(FieldUtil.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);
    }

    @Override
    public void resetToParamaterizedRotation(Rotation2d rotation2d) {
        this.resetRotation(rotation2d);
    }

    @Override
    public void resetRobotTranslation(Translation2d translation2d) {
        this.resetTranslation(translation2d);
    }

    @Override
    public void updateSimState() {
        this.updateSimState(ProjectConstants.DEFAULT_PERIOD_SECS, 13.00);
    }

    public void updateModuleInputs(ModuleIOInputs... inputs) {
        for (int i = 0; i < 4; i++) {
            var moduleMap = signalsMap.get(i);

            inputs[i].driveSupplyCurrentAmps =
                    moduleMap.get("driveSupplyCurrentAmps").getValueAsDouble();
            inputs[i].driveStatorCurrentAmps =
                    moduleMap.get("driveStatorCurrentAmps").getValueAsDouble();
            inputs[i].driveAppliedVolts = moduleMap.get("driveAppliedVolts").getValueAsDouble();
            inputs[i].driveTemperature = moduleMap.get("driveTemperature").getValueAsDouble();

            inputs[i].steerSupplyCurrentAmps =
                    moduleMap.get("steerSupplyCurrentAmps").getValueAsDouble();
            inputs[i].steerStatorCurrentAmps =
                    moduleMap.get("steerStatorCurrentAmps").getValueAsDouble();
            inputs[i].steerAppliedVolts = moduleMap.get("steerAppliedVolts").getValueAsDouble();
            inputs[i].steerTemperature = moduleMap.get("steerTemperature").getValueAsDouble();
        }
    }

    @Override
    public void refreshData() {
        for (int i = 0; i < 4; i++) {
            var moduleMap = signalsMap.get(i);
            BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[] {}));
        }
    }

    //Wrapers to the CTRE swerveDrivetrain
    public SwerveModulePosition[] getModulesPositions() {
        return this.getStateCopy().ModulePositions;
    }


    public Rotation2d getYaw() {
        return this.getState().Pose.getRotation();
    }

    public void disableMotors() {
        for (int i = 0; i < 4; i++) {
            this.getModule(i).getDriveMotor().disable();
            this.getModule(i).getSteerMotor().disable();
        }
    }

    //For testing
    public TalonFX[] getMotorsByIds(int... ids) {
        if (ids == null || ids.length == 0) return new TalonFX[0];

        Set<Integer> requested = new HashSet<>();
        for (int id : ids) requested.add(id);

        Map<Integer, TalonFX> found = new HashMap<>(requested.size());

        for (int i = 0; i < 4 && found.size() < requested.size(); i++) {
            TalonFX drive = this.getModule(i).getDriveMotor();
            TalonFX steer = this.getModule(i).getSteerMotor();

            int dId = drive.getDeviceID();
            int sId = steer.getDeviceID();

            if (requested.contains(dId)) found.putIfAbsent(dId, drive);
            if (requested.contains(sId)) found.putIfAbsent(sId, steer);
        }

        List<TalonFX> out = new ArrayList<>(ids.length);
        Set<Integer> missing = new LinkedHashSet<>();
        
        for (int id : ids) {
            TalonFX m = found.get(id);
            if (m != null) out.add(m);
            else missing.add(id);
        }

        if (!missing.isEmpty()) {
            DriverStation.reportWarning("Some requested motor IDs were not found: " + missing, false);
        }

        return out.toArray(new TalonFX[0]);
    }

    public TalonFX getMotorById(int id) {
        TalonFX[] arr = getMotorsByIds(id);
        if (arr.length == 0) {
            throw new IllegalArgumentException("Motor with device ID " + id + " not found.");
        }
        return arr[0];
    }
    

    
}
