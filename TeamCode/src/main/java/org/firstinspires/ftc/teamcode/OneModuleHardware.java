package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputEx;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;


public class OneModuleHardware {

    public DcMotor LeftM1  = null;
    public DcMotor RightM1  = null;
    public AnalogInput EncoderM = null;
    public double encoderOffset = 0;


    HardwareMap hwMap           =  null;

    public OneModuleHardware(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        LeftM1 = hwMap.get(DcMotor.class, "LeftM1");
        RightM1 = hwMap.get(DcMotor.class, "RightM1");
        EncoderM = hwMap.get(AnalogInput.class, "EncoderM1");


        LeftM1.setDirection(DcMotor.Direction.FORWARD);
        RightM1.setDirection(DcMotor.Direction.FORWARD);


        LeftM1.setPower(0);
        RightM1.setPower(0);


        LeftM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LeftM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void freeze() {

        LeftM1.setPower(0);
        RightM1.setPower(0);

    }

    public double getAngle()
    {
        double rawAngle = (EncoderM.getVoltage()) * .4 * Math.PI - encoderOffset;
        if (rawAngle > Math.PI)
            rawAngle -= 2 * Math.PI;
        if (rawAngle < -Math.PI)
            rawAngle += 2 * Math.PI;

        return rawAngle;
    }

}
