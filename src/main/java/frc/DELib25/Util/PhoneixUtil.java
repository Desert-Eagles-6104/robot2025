// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class PhoneixUtil {
     public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
        StatusCode code = function.get();
        int tries = 0;
        while (code != StatusCode.OK && tries < numTries) {
            DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
            code = function.get();
            tries++;
        }
        if (code != StatusCode.OK) {
            DriverStation.reportError("Failed to execute phoenix pro api call after " + numTries + " attempts", false);
            return false;
        }
        return true;
    }

    public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
        return checkErrorAndRetry(function, 5);
    }
}
