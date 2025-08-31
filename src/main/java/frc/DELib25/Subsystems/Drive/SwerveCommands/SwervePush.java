package frc.DELib25.Subsystems.Drive.SwerveCommands;

import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.DELib25.Subsystems.Drive.DriveState;
import frc.DELib25.Subsystems.Drive.SwerveSubsystem;

public class SwervePush extends Command {
	private static boolean mySwitch = false;
    private final SwerveSubsystem swerve;
    private final BooleanSupplier exit;

    private DriveState originalState;

    public SwervePush(SwerveSubsystem swerve, BooleanSupplier exitCondition) {
        this.swerve = Objects.requireNonNull(swerve);
        this.exit = exitCondition != null ? exitCondition : () -> false;
        addRequirements(swerve);
        setName("SwervePush");
    }

    @Override
    public void initialize() {

        originalState = swerve.getWantedState();

        for (SwerveModule<TalonFX, TalonFX, CANcoder> m : swerve.getIO().getModules()) {
            m.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
            m.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        }
		
        swerve.setWantedState(DriveState.IDLE);
        swerve.getIO().setSwerveState(new SwerveRequest.Idle());

        DriverStation.reportWarning("[Swerve] Push Mode: ENABLED", false);
    }

    @Override
    public void execute() {
		System.out.println("Push Mode Active with value: "+mySwitch);
        swerve.getIO().setSwerveState(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return exit.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setWantedState(originalState);
		for (SwerveModule<TalonFX, TalonFX, CANcoder> m : swerve.getIO().getModules()) {
			m.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
			m.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
		}
        DriverStation.reportWarning("[Swerve] Push Mode: " + (interrupted ? "INTERRUPTED" : "DISABLED"), false);
    }

    public static void bind(SwerveSubsystem swerve) {
        GenericEntry entry = Shuffleboard.getTab("Swerve")
            .add("Push swerve", false) 
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        new Trigger(() -> entry.getBoolean(false))
            .whileTrue(new SwervePush(swerve, () -> !entry.getBoolean(false)));
	}
}
