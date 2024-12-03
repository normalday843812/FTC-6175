package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Full Teleop for Comp", group="GYATTTTTTTTTTTTTTTT")
public class FullThing extends OpMode {
    HardwareMapThing robot = new HardwareMapThing();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("init?", "init");
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        robot.FLMotor.setPower(leftFrontPower);
        robot.FRMotor.setPower(rightFrontPower);
        robot.BLMotor.setPower(leftBackPower);
        robot.BRMotor.setPower(rightBackPower);

        telemetry.addData("fl,fr", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("bl,br", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addLine("SKIBIDI RIZZZZZZZ");
        telemetry.update();
    }
}