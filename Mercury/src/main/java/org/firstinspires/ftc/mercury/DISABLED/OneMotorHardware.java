package org.firstinspires.ftc.mercury.DISABLED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class OneMotorHardware
{

    public DcMotor  LM0  = null;
    HardwareMap hwMap =  null;

    public OneMotorHardware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        LM0 = hwMap.get(DcMotor.class, "LM0");
        LM0.setDirection(DcMotor.Direction.REVERSE);

        LM0.setPower(0);
        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders() {

        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

