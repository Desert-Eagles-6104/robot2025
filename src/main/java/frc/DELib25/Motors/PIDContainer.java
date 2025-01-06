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

        public PIDContainer(double kp, double ki, double kd, String headingType)
    {
        this.kS = 0.0;
        this.kV = 0.0;
        this.kA = 0.0;
        this.kG = 0.0;
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.headingType = headingType;
    }

    public PIDContainer(double kS, double kV, double kA)
    {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = 0.0;
        this.kP = 0.0;
        this.kI = 0.0;
        this.kD = 0.0;
    }

        public PIDContainer(double kS, double kV, double kA, double kG)
    {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.kP = 0.0;
        this.kI = 0.0;
        this.kD = 0.0;
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
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
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
/* 
    public static SparkPIDController toSparkPIDController(PIDContainer pidContainer, CANSparkBase controller){
        SparkPIDController sparkPIDController = controller.getPIDController();
        sparkPIDController.setP(pidContainer.kP);
        sparkPIDController.setI(pidContainer.kI);
        sparkPIDController.setD(pidContainer.kD);
        sparkPIDController.setOutputRange(-1, 1);
        return sparkPIDController;
    }*/
}