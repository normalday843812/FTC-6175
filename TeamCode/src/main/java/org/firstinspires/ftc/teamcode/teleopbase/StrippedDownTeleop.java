package org.firstinspires.ftc.teamcode.teleopbase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="StrippedDownTeleop", group="Simple")
public class StrippedDownTeleop extends LinearOpMode {

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private Servo clawServo;

    private boolean clawOpen = false;
    private boolean prevA = false;

    @Override
    public void runOpMode() {
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        if (FLMotor != null) FLMotor.setDirection(DcMotor.Direction.FORWARD);
        if (FRMotor != null) FRMotor.setDirection(DcMotor.Direction.REVERSE);
        if (BLMotor != null) BLMotor.setDirection(DcMotor.Direction.FORWARD);
        if (BRMotor != null) BRMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FLMotor.setPower(frontLeftPower);
            BLMotor.setPower(backLeftPower);
            FRMotor.setPower(frontRightPower);
            BRMotor.setPower(backRightPower);

            if(gamepad2.a && !prevA) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? 0.39 : 0.14);
            }
            prevA = gamepad2.a;

            telemetry.addData("Claw", clawOpen?"Open":"Closed");
            telemetry.update();
        }
    }
}
