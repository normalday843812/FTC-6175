package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** @noinspection BooleanMethodIsAlwaysInverted*/

// 90% of this stuff is just for pure sanity and to make it as robust as possible
public class HardwareMapThing {
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;
    public DcMotor BucketMotor0 = null;
    public DcMotor BucketMotor1 = null;
    public Servo clawServo = null;
    public Servo clawRollServo = null;
    public Servo clawPitchServo = null;
    public Servo ArmServo0 = null;
    public Servo ArmServo1 = null;
    public Servo ArmPitchServo0 = null;
    public Servo ArmPitchServo1 = null;
    public static final double CLAW_MIN_POSITION = 0.39;
    public static final double CLAW_MAX_POSITION = 0.14;
    public static final double DEADZONE = 0.1;
    public static final double MAX_SERVO_SPEED = 1.0;
    public HardwareMap hwMap = null;
    private boolean hardwareError = false;
    private final StringBuilder errorMessage = new StringBuilder();

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        FLMotor = safeGetDevice(DcMotor.class, "FLMotor");
        FRMotor = safeGetDevice(DcMotor.class, "FRMotor");
        BLMotor = safeGetDevice(DcMotor.class, "BLMotor");
        BRMotor = safeGetDevice(DcMotor.class, "BRMotor");
        BucketMotor0 = safeGetDevice(DcMotor.class, "BucketMotor0");
        BucketMotor1 = safeGetDevice(DcMotor.class, "BucketMotor1");

        clawServo = safeGetDevice(Servo.class, "clawServo");
        clawRollServo = safeGetDevice(Servo.class, "clawRollServo");
        clawPitchServo = safeGetDevice(Servo.class, "clawPitchServo");
        ArmServo0 = safeGetDevice(Servo.class, "ArmServo0");
        ArmServo1 = safeGetDevice(Servo.class, "ArmServo1");
        ArmPitchServo0 = safeGetDevice(Servo.class, "ArmPitchServo0");
        ArmPitchServo1 = safeGetDevice(Servo.class, "ArmPitchServo1");

        if (!hardwareError) {
            configureMotorsAndServos();
        }
    }

    private void configureMotorsAndServos() {
        if (FLMotor != null) FLMotor.setDirection(DcMotor.Direction.FORWARD);
        if (FRMotor != null) FRMotor.setDirection(DcMotor.Direction.REVERSE);
        if (BLMotor != null) BLMotor.setDirection(DcMotor.Direction.FORWARD);
        if (BRMotor != null) BRMotor.setDirection(DcMotor.Direction.REVERSE);

        if (BucketMotor0 != null) {
            BucketMotor0.setDirection(DcMotor.Direction.REVERSE);
            BucketMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BucketMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (BucketMotor1 != null) {
            BucketMotor1.setDirection(DcMotor.Direction.FORWARD);
            BucketMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BucketMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        safeSetZeroPowerBehavior(FLMotor, DcMotor.ZeroPowerBehavior.BRAKE);
        safeSetZeroPowerBehavior(FRMotor, DcMotor.ZeroPowerBehavior.BRAKE);
        safeSetZeroPowerBehavior(BLMotor, DcMotor.ZeroPowerBehavior.BRAKE);
        safeSetZeroPowerBehavior(BRMotor, DcMotor.ZeroPowerBehavior.BRAKE);

        if (clawServo != null) clawServo.setPosition(CLAW_MIN_POSITION);
        if (clawRollServo != null) clawRollServo.setPosition(0.5);
        if (clawPitchServo != null) clawPitchServo.setPosition(0.5);
    }

    private <T> T safeGetDevice(Class<? extends T> classOrInterface, String deviceName) {
        try {
            T device = hwMap.get(classOrInterface, deviceName);
            if (device == null) {
                hardwareError = true;
                errorMessage.append("Device not found: ").append(deviceName).append("\n");
            }
            return device;
        } catch (Exception e) {
            hardwareError = true;
            errorMessage.append("Exception getting ").append(deviceName)
                    .append(": ").append(e.getMessage()).append("\n");
            return null;
        }
    }

    /** @noinspection SameParameterValue*/
    private void safeSetZeroPowerBehavior(DcMotor motor, DcMotor.ZeroPowerBehavior behavior) {
        if (motor != null) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public boolean hasHardwareError() {
        return hardwareError;
    }

    public String getErrorMessage() {
        return errorMessage.toString();
    }

    public void reportStatusToTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        if (hardwareError) {
            telemetry.addLine("Hardware Error Detected:");
            telemetry.addLine(getErrorMessage());
        } else {
            telemetry.addLine("Hardware OK");
        }
    }
}