package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="absEncTest", group="Pushbot")
//@Disabled
public class absEncTest extends LinearOpMode {

    absEncHW r = new absEncHW();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addData("DeviceName", r.monkey.getDeviceName());
                telemetry.addData("ConnectionInfo", r.monkey.getConnectionInfo());
                telemetry.addData("getPulseWidthOutputTime", r.monkey.getPulseWidthOutputTime());
                telemetry.addData("getPulseWidthPeriod", r.monkey.getPulseWidthPeriod());
                telemetry.update();
            }
        }

    }
}