// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.DELib25.Motors;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// /** Add your docs here. */
// public class SparkMaxFactory {
    
//     public static CANSparkMax createSparkMax(MotorConstants motorConstants, boolean useDefaultConfig){
//         CANSparkMax spark = createSparkMax(motorConstants.id);
//         if(useDefaultConfig){
//             spark.setCANMaxRetries(5);
//             spark.setIdleMode(MotorConstants.toIdleMode(motorConstants.isBrake));
//             spark.setInverted(motorConstants.CounterClockwisePositive);
//             spark.enableVoltageCompensation(12);
//             spark.setSmartCurrentLimit(40);
//             spark.burnFlash();
//         }
//         return spark;
//     }

//     public static CANSparkMax createSlaveSparkMax(CANSparkMax mater, int slaveId, boolean opposeMasterDirection){
//         CANSparkMax spark = createSparkMax(slaveId);
//         spark.follow(mater, opposeMasterDirection);
//         spark.burnFlash();
//         return spark;
//     }
    
//     public static CANSparkMax createSparkMax(int id){
//         CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
//         spark.restoreFactoryDefaults();
//         spark.clearFaults();
//         return spark;
//     }
// }
