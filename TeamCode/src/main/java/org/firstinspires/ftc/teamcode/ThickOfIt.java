package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ThickOfIt {
    private final AndroidSoundPool androidSoundPool;
    private int currentSoundIndex;
    private static final int TOTAL_SOUNDS = 33;
    private static final double SOUND_DURATION_MS = 5000;
    private final ElapsedTime soundTimer;
    private boolean isPlaying;

    public ThickOfIt() {
        androidSoundPool = new AndroidSoundPool();
        androidSoundPool.initialize(SoundPlayer.getInstance());
        currentSoundIndex = 1;
        soundTimer = new ElapsedTime();
        isPlaying = false;
    }

    public void preloadSounds() {
        for (int i = 1; i <= TOTAL_SOUNDS; i++) {
            String soundFileName = i + ".mp3";
            boolean success = androidSoundPool.preloadSound(soundFileName);

            if (!success) {
                System.out.println("Failed to preload sound: " + soundFileName);
            } else {
                System.out.println("Sound preloaded successfully: " + soundFileName);
            }
        }
    }

    public void startSoundSequence() {
        if (!isPlaying) {
            isPlaying = true;
            currentSoundIndex = 1;
            playCurrentSound();
            soundTimer.reset();
        }
    }

    // Update method - should be called in the OpMode loop
    public void update() {
        if (isPlaying && soundTimer.milliseconds() >= SOUND_DURATION_MS) {
            currentSoundIndex++;
            if (currentSoundIndex <= TOTAL_SOUNDS) {
                playCurrentSound();
                soundTimer.reset();
            } else {
                isPlaying = false;
                currentSoundIndex = 1;
            }
        }
    }

    // Play the current sound
    private void playCurrentSound() {
        String soundFileName = currentSoundIndex + ".mp3";
        boolean success = androidSoundPool.play(soundFileName);

        if (success) {
            System.out.println("Playing sound: " + soundFileName);
        } else {
            System.out.println("Failed to play sound: " + soundFileName);
            isPlaying = false;  // Stop sequence if playback fails
        }
    }

    // Stop the sequence
    public void stopSequence() {
        isPlaying = false;
        currentSoundIndex = 1;
    }

    // Get current sound index
    public int getCurrentIndex() {
        return currentSoundIndex;
    }

    // Check if sequence is playing
    public boolean isPlaying() {
        return isPlaying;
    }

    // Cleanup resources
    public void release() {
        androidSoundPool.close();
        System.out.println("Resources released.");
    }
}