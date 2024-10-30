package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
public class AutoPrevious extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        driveForward(1.0);

        sleep(3000);

        stopDriving();

        driveLeft(1.0);

        stopDriving();
    }

    private void driveForward(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void driveLeft(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

}
