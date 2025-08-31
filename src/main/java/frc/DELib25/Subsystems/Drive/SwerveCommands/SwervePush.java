package frc.DELib25.Subsystems.Drive.SwerveCommands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.Drive.DriveState;
import frc.DELib25.Subsystems.Drive.SwerveSubsystem;

public class SwervePush extends Command {
    private final SwerveSubsystem swerve;
    private DriveState originalState;
    private BooleanSupplier isFinished;

    public SwervePush(SwerveSubsystem swerve, BooleanSupplier isFinished) {
        this.swerve = swerve;
        this.isFinished = isFinished;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        originalState = swerve.getWantedState();
        swerve.setWantedState(DriveState.IDLE); // your code should do nothing in IDLE

        // Set Coast once so there’s no electrical braking
        for (SwerveModule<TalonFX, TalonFX, CANcoder> m : swerve.getIO().getModules()) {
            m.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
            m.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
            // Immediately neutral outputs
            m.getDriveMotor().setControl(new NeutralOut());
            m.getSteerMotor().setControl(new NeutralOut());
        }
        DriverStation.reportWarning("Push Mode: ENABLED", false);
    }

    @Override
    public void execute() {
        if (isFinished.getAsBoolean()) {
            this.end(false);
            this.cancel();
        }
        // Keep everything neutral so no loop elsewhere can “grab” the motors
        for (SwerveModule<TalonFX, TalonFX, CANcoder> m : swerve.getIO().getModules()) {
            m.getDriveMotor().setControl(new NeutralOut());
            m.getSteerMotor().setControl(new NeutralOut());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Restore your normal neutral mode (change if you normally use Coast)
        for (SwerveModule<TalonFX, TalonFX, CANcoder> m : swerve.getIO().getModules()) {
            m.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
            m.getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        }
        swerve.setWantedState(originalState);
        DriverStation.reportWarning("Push Mode: DISABLED", false);
    }

    public static void bind(SwerveSubsystem swerve) {
        SmartDashboard.putBoolean("Push Mode", false);
		SmartDashboard.putData("Push Mode Control", 
            new SwervePush(swerve, () -> SmartDashboard.getBoolean("Push Mode", false))
        );
    }
}