package org.firstinspires.ftc.teamcode.autonomousbase.messages;

import org.firstinspires.ftc.teamcode.autonomousbase.GoBildaPinpointDriver;

public final class PinpointLocalizerInputsMessage {
    public long timestamp;

    // Position data in millimeters
    public double xPosition;
    public double yPosition;
    public double heading;

    // Velocity data in mm/sec and rad/sec
    public double xVelocity;
    public double yVelocity;
    public double headingVelocity;

    // Raw encoder values and device status
    public int xEncoderValue;
    public int yEncoderValue;
    public GoBildaPinpointDriver.DeviceStatus deviceStatus;

    // Loop timing information
    public int loopTime;
    public double frequency;

    public PinpointLocalizerInputsMessage(GoBildaPinpointDriver pinpoint) {
        this.timestamp = System.nanoTime();

        // Capture current position data
        this.xPosition = pinpoint.getPosX();
        this.yPosition = pinpoint.getPosY();
        this.heading = pinpoint.getHeading();

        // Capture velocity data
        this.xVelocity = pinpoint.getVelX();
        this.yVelocity = pinpoint.getVelY();
        this.headingVelocity = pinpoint.getHeadingVelocity();

        // Capture raw encoder values
        this.xEncoderValue = pinpoint.getEncoderX();
        this.yEncoderValue = pinpoint.getEncoderY();

        // Capture device status and performance metrics
        this.deviceStatus = pinpoint.getDeviceStatus();
        this.loopTime = pinpoint.getLoopTime();
        this.frequency = pinpoint.getFrequency();
    }
}