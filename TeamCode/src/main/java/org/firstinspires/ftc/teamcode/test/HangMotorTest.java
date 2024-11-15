package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(group = "tuning")
public class HangMotorTest extends LinearOpMode {
    public static double HANG_POWER = 1.0;
    public static int TARGET_POSITION = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx hangMotorL = hardwareMap.get(DcMotorEx.class, "hangMotorLeft");
        DcMotorEx hangMotorR = hardwareMap.get(DcMotorEx.class, "hangMotorRight");

        hangMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        // Test extension
        hangMotorL.setTargetPosition(TARGET_POSITION);
        hangMotorR.setTargetPosition(-TARGET_POSITION);

        hangMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hangMotorL.setPower(HANG_POWER);
        hangMotorR.setPower(HANG_POWER);

        while (opModeIsActive() &&
                (hangMotorL.isBusy() || hangMotorR.isBusy())) {
            telemetry.addData("Left Position", hangMotorL.getCurrentPosition());
            telemetry.addData("Right Position", hangMotorR.getCurrentPosition());
            telemetry.addData("Left Power", hangMotorL.getPower());
            telemetry.addData("Right Power", hangMotorR.getPower());
            telemetry.update();
        }

        sleep(1000);

        // Test retraction
        hangMotorL.setTargetPosition(0);
        hangMotorR.setTargetPosition(0);

        hangMotorL.setPower(HANG_POWER);
        hangMotorR.setPower(HANG_POWER);

        while (opModeIsActive() &&
                (hangMotorL.isBusy() || hangMotorR.isBusy())) {
            telemetry.addData("Left Position", hangMotorL.getCurrentPosition());
            telemetry.addData("Right Position", hangMotorR.getCurrentPosition());
            telemetry.addData("Left Power", hangMotorL.getPower());
            telemetry.addData("Right Power", hangMotorR.getPower());
            telemetry.update();
        }
    }
}