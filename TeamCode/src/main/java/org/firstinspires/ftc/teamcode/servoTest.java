package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="TeleOp")
public class servoTest extends LinearOpMode {
    boolean previousDPadUpState = false;
    boolean previousDPadDownState = false;

    double servoPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawServo = hardwareMap.get(Servo.class, "testServo");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !previousDPadUpState) {
                servoPosition += 0.03;
                if (servoPosition > 1.0) {
                    servoPosition = 1.0;
                }
            }

            if (gamepad1.dpad_down && !previousDPadDownState) {
                servoPosition -= 0.03;
                if (servoPosition < 0.0) {
                    servoPosition = 0.0;
                }
            }

            clawServo.setPosition(servoPosition);

            previousDPadUpState = gamepad1.dpad_up;
            previousDPadDownState = gamepad1.dpad_down;

            telemetry.addData("Servo Position", "%.3f", servoPosition);
            telemetry.update();
        }
    }
}
