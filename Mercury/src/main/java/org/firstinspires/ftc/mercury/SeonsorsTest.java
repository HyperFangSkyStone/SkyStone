package org.firstinspires.ftc.mercury;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name="SEONOSORES TEST", group="1")
@Disabled
public class SeonsorsTest extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();

    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive.init(hardwareMap);


        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Left Range:", tankDrive.LeftRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Range:", tankDrive.RightRange.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}