package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Controller Test", group="TeleOp")
public class ControllerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "init finished");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);
            telemetry.addData("Left Stick", "X: %.2f, Y: %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Right Stick", "X: %.2f, Y: %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
            telemetry.addData("DPad Up", gamepad1.dpad_up);
            telemetry.addData("DPad Down", gamepad1.dpad_down);
            telemetry.addData("DPad Left", gamepad1.dpad_left);
            telemetry.addData("DPad Right", gamepad1.dpad_right);
            telemetry.addData("Back", gamepad1.back);
            telemetry.addData("Start/Options", gamepad1.options);
            telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
            telemetry.addData("Right Stick Button", gamepad1.right_stick_button);

            telemetry.addData("Circle", gamepad1.circle);
            telemetry.addData("Cross", gamepad1.cross);
            telemetry.addData("Square", gamepad1.square);
            telemetry.addData("Triangle", gamepad1.triangle);

            telemetry.update();
        }
    }
}
