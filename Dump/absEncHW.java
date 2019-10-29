package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

public class absEncHW {

    public PWMOutputImpl monkey = null;
    HardwareMap hwMap           =  null;


    public absEncHW()
    {

    };

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        monkey = hwMap.get(PWMOutputImpl.class , "monkey");
    }
}
