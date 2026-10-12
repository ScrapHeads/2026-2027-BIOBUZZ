package org.firstinspires.ftc.teamcode.robotTests.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class VisionSubsystemTest {

    public VisionSubsystem createVisionSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withLimelight("limelight")
                .build();

        MultipleTelemetry mockTelemetry = new MultipleTelemetry();
        return new VisionSubsystem(hardwareMap, mockTelemetry, FtcDashboard.getInstance());
    }

    @Test
    @DisplayName("Test VisionSubsystem creation with mock Limelight")
    public void testVisionSubsystemCreation() {
        VisionSubsystem visionSubsystem = createVisionSubsystem();
        assertNotNull(visionSubsystem);
    }
}
