package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private static double kP, kI, kD;

    public PIDController(double kP)
    {
        this.kP = kP;
        kI = 0;
        kD = 0;
    }

    public PIDController(double kP, double kD)
    {
        this.kP = kP;
        kI = 0;
        this.kD = kD;
    }

    public PIDController(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public static double PIDOutput(double targetTick, double error, double prevError, double time, double prevTime, double floor)
    {
        double p = error / targetTick * kP;
        double d = ((error - prevError) / (time - prevTime)) /targetTick * kD;
        double i = error * (time - prevTime) * kI;

        double output = p + i + d;

        if (output < 0 && output > -floor)
            output = -floor;
        if (output > 0 && output < floor)
            output = floor;

        return output;
    }

}
