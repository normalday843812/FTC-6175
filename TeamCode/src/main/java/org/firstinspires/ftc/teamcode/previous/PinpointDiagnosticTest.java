package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp(name = "Pinpoint Diagnostic Test", group = "Diagnostics")
public class PinpointDiagnosticTest extends OpMode {

    private I2cDeviceSynch deviceClient;
    private static final int PINPOINT_I2C_ADDRESS = 0x31; // 7-bit address

    @Override
    public void init() {
        try {
            deviceClient = hardwareMap.get(I2cDeviceSynch.class, "i2cDevice");
            deviceClient.setI2cAddress(I2cAddr.create7bit(PINPOINT_I2C_ADDRESS));
            deviceClient.engage();

            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Initialization Failed");
            telemetry.addData("Error", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if (deviceClient != null) {
            try {
                // Read Device ID
                byte[] deviceIdBytes = deviceClient.read(1, 4);
                int deviceId = java.nio.ByteBuffer.wrap(deviceIdBytes).order(java.nio.ByteOrder.LITTLE_ENDIAN).getInt();

                telemetry.addData("Device ID", deviceId);

                // Read Device Status
                byte[] statusBytes = deviceClient.read(3, 4);
                int deviceStatus = java.nio.ByteBuffer.wrap(statusBytes).order(java.nio.ByteOrder.LITTLE_ENDIAN).getInt();

                // Interpret Device Status
                boolean isReady = (deviceStatus & 1) != 0;
                boolean isCalibrating = (deviceStatus & (1 << 1)) != 0;
                boolean xPodNotDetected = (deviceStatus & (1 << 2)) != 0;
                boolean yPodNotDetected = (deviceStatus & (1 << 3)) != 0;

                telemetry.addData("Device Status", deviceStatus);
                telemetry.addData("Is Ready", isReady);
                telemetry.addData("Is Calibrating", isCalibrating);
                telemetry.addData("X Pod Not Detected", xPodNotDetected);
                telemetry.addData("Y Pod Not Detected", yPodNotDetected);

                // Read X and Y Positions
                float xPosition = readFloatFromRegister(8);
                float yPosition = readFloatFromRegister(9);

                telemetry.addData("X Position (mm)", xPosition);
                telemetry.addData("Y Position (mm)", yPosition);

                // Read Heading
                float heading = readFloatFromRegister(10);
                telemetry.addData("Heading (rad)", heading);

                // Read Loop Time
                int loopTime = readUInt32FromRegister(5);
                telemetry.addData("Loop Time (µs)", loopTime);

            } catch (Exception e) {
                telemetry.addData("Error", "Failed to communicate with Pinpoint");
                telemetry.addData("Exception", e.getMessage());
            }
        } else {
            telemetry.addData("Error", "Device client is null");
        }
        telemetry.update();
    }

    private int readUInt32FromRegister(int registerAddress) {
        byte[] bytes = deviceClient.read(registerAddress, 4);
        return java.nio.ByteBuffer.wrap(bytes).order(java.nio.ByteOrder.LITTLE_ENDIAN).getInt();
    }

    private float readFloatFromRegister(int registerAddress) {
        byte[] bytes = deviceClient.read(registerAddress, 4);
        return java.nio.ByteBuffer.wrap(bytes).order(java.nio.ByteOrder.LITTLE_ENDIAN).getFloat();
    }
}
