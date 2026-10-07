package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.config.VisionConstants.START_PIPELINE;

import com.qualcomm.hardware.limelightvision.Limelight3A;

public class VisionStateManager {
    /**
     * Controls which Limelight mode/region is currently being used:
     * - game-element scan regions
     * - AprilTag/localization mode
     */

    private final Limelight3A limelight;

    private ScanRegions region;

    private int currentPipeline;

    public VisionStateManager(Limelight3A limelight) {
        this.limelight = limelight;
        this.region = ScanRegions.TOP_LEFT;
        this.currentPipeline = START_PIPELINE;
    }

    public void setCurrentPipeline(int newPipeline) {
        this.currentPipeline = newPipeline;
        limelight.pipelineSwitch(currentPipeline);
    }

    public int getCurrentPipeline() {
        return currentPipeline;
    }

    public ScanRegions getRegion () {
        return region;
    }

    public void advanceRegion () {
        switch (region) {
            case TOP_LEFT:
                region = ScanRegions.TOP_RIGHT;
                break;
            case TOP_RIGHT:
                region = ScanRegions.BOTTOM_LEFT;
                break;
            case BOTTOM_LEFT:
                region = ScanRegions.BOTTOM_RIGHT;
                break;
            case BOTTOM_RIGHT:
                region = ScanRegions.TOP_LEFT;
                break;
        }
    }
}
