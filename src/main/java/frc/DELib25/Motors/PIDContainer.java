package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.revrobotics.CANSparkBase;
// import com.revrobotics.SparkPIDController;

public class PIDContainer
{
    //#region FF Values
    public double kS;
    public double kV;
    public double kA;
    public double kG;
    //#endregion FF values

    //#region PID Values
    public double kP;
    public double kI;
    public double kD;
    //#endregion PID Values

    public String headingType;

    public GravityTypeValue gravityTypeValue = GravityTypeValue.Elevator_Static; //only for talonFX

    public PIDContainer(double kP, double kI, double kD, String headingType)
    {
        this(0.0, 0.0, 0.0, 0.0, kP, kI, kD);
        this.headingType = headingType;
    }

    public PIDContainer(double kS, double kV, double kA)
    {
        this(kS, kV, kA, 0.0);
    }

    public PIDContainer(double kS, double kV, double kA, double kG)
    {
        this(kS, kV, kA, kG, 0.0, 0.0, 0.0);
    }

    public PIDContainer(double kS, double kV, double kA, double kG, double kP, double kI)
    {
        this(kS, kV, kA, kG, kP, kI, 0.0);
    }

    public PIDContainer(double kS, double kV, double kA, double kG, double kP, double kI, double kD)
    {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDContainer(double kS, double kV, double kA, double kG, double kP, double kI, double kD, GravityTypeValue gravityTypeValue)
    {
        this(kS, kV, kA, kG, kP, kI, kD);
        this.gravityTypeValue = gravityTypeValue;
    }

    public static Slot0Configs toSlot0Configs(PIDContainer pidContainer){
        return new Slot0Configs()
        .withKS(pidContainer.kS)
        .withKV(pidContainer.kV)
        .withKA(pidContainer.kA)
        .withKG(pidContainer.kG)
        .withKP(pidContainer.kP)
        .withKI(pidContainer.kI)
        .withKD(pidContainer.kD)
        .withGravityType(pidContainer.gravityTypeValue);
    }

    public static Slot1Configs toSlot1Configs(PIDContainer pidContainer){
        return new Slot1Configs()
        .withKS(pidContainer.kS)
        .withKV(pidContainer.kV)
        .withKA(pidContainer.kA)
        .withKG(pidContainer.kG)
        .withKP(pidContainer.kP)
        .withKI(pidContainer.kI)
        .withKD(pidContainer.kD)
        .withGravityType(pidContainer.gravityTypeValue);
    }
    
}