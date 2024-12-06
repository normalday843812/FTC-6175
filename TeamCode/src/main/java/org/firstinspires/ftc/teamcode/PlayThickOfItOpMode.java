package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="PlayThickOfItOpMode", group="TeleOp")
public class PlayThickOfItOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ThickOfIt thickOfIt = new ThickOfIt();
        thickOfIt.preloadSounds();

        telemetry.addData("Status", "init finished");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        thickOfIt.startSoundSequence();
        telemetry.addData("Status", "Playing Sequence");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            thickOfIt.update();

            telemetry.addData("playing this one:", thickOfIt.isPlaying());
            telemetry.addData("sound index:", thickOfIt.getCurrentIndex());
            telemetry.update();
        }

        thickOfIt.stopSequence();
        thickOfIt.release();

        telemetry.addData("status:", "stopped; resources released (you'll never even see this lmao)");
        telemetry.update();
    }
}
