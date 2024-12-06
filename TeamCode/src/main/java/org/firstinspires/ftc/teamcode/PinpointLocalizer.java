package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import org.firstinspires.ftc.teamcode.messages.PinpointLocalizerInputsMessage;

public class PinpointLocalizer implements Localizer {
    // The conversion factor from millimeters (Pinpoint) to inches (Roadrunner)
    private static final double MM_PER_INCH = 25.4;

    // Our Pinpoint driver instance
    private final GoBildaPinpointDriver pinpoint;

    // State tracking variables
    private double lastXMM = 0;
    private double lastYMM = 0;
    private double lastHeading = 0;
    private boolean initialized = false;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    @Override
    public Twist2dDual<Time> update() {
        // Get fresh data from Pinpoint
        pinpoint.update();

        // Record all inputs for debugging and analysis
        FlightRecorder.write("PINPOINT_LOCALIZER_INPUTS",
                new PinpointLocalizerInputsMessage(pinpoint));

        // Get current positions from Pinpoint
        double currentXMM = pinpoint.getPosX();
        double currentYMM = pinpoint.getPosY();
        double currentHeading = pinpoint.getHeading();

        // Handle first update case
        if (!initialized) {
            initialized = true;
            lastXMM = currentXMM;
            lastYMM = currentYMM;
            lastHeading = currentHeading;

            // Return zero motion for initial update
            return new Twist2dDual<>(
                    new Vector2dDual<>(
                            DualNum.constant(0.0, 2),  // x: no displacement, no velocity
                            DualNum.constant(0.0, 2)   // y: no displacement, no velocity
                    ),
                    DualNum.constant(0.0, 2)       // angle: no rotation, no velocity
            );
        }

        // Calculate changes in position since last update
        double dxMM = currentXMM - lastXMM;
        double dyMM = currentYMM - lastYMM;
        double dh = currentHeading - lastHeading;

        // Get velocity data from Pinpoint
        double xVelMM = pinpoint.getVelX();
        double yVelMM = pinpoint.getVelY();
        double headingVel = pinpoint.getHeadingVelocity();

        // Update stored positions for next calculation
        lastXMM = currentXMM;
        lastYMM = currentYMM;
        lastHeading = currentHeading;

        // Create the Twist2dDual representing both motion and velocity
        return new Twist2dDual<>(
                // Linear motion component
                new Vector2dDual<>(
                        // X component: position change and velocity in inches
                        new DualNum<>(new double[] {
                                dxMM / MM_PER_INCH,    // Convert position change to inches
                                xVelMM / MM_PER_INCH   // Convert velocity to inches/sec
                        }),
                        // Y component: position change and velocity in inches
                        new DualNum<>(new double[] {
                                dyMM / MM_PER_INCH,    // Convert position change to inches
                                yVelMM / MM_PER_INCH   // Convert velocity to inches/sec
                        })
                ),
                // Angular motion component (already in radians)
                new DualNum<>(new double[] {
                        dh,          // Change in heading
                        headingVel   // Angular velocity
                })
        );
    }
}