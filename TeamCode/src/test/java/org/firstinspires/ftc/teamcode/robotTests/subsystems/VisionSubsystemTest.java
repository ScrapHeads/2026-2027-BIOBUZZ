package org.firstinspires.ftc.teamcode.robotTests.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class VisionSubsystemTest {

    public VisionSubsystem createVisionSubsystem() {
        TestUtils.MockHardwareMap hardwareMap = new TestUtils.HardwareMapBuilder()
                .withLimelight("limelight")
                .build();

        return TestUtils.createSubsystem(
                hm -> new VisionSubsystem(hm, TestUtils.getMultipleTelemetry(),
                        TestUtils.startMockDashboard()), hardwareMap);
    }

    @Test
    @DisplayName("Test VisionSubsystem creation with mock Limelight")
    public void testVisionSubsystemCreation() {
        VisionSubsystem visionSubsystem = createVisionSubsystem();
        assertNotNull(visionSubsystem);
    }

    @Test
    @DisplayName("Test Missing Hardware Handling")
    public void testVisionSubsystemMissingHardwareHandling() {
        TestUtils.MockHardwareMap emptyMap = new TestUtils.MockHardwareMap();
        assertThrows(
                IllegalArgumentException.class,
                () -> new TransferSubsystem(emptyMap),
                "Subsystem creation should fail if required motor or servo names are missing from HardwareMap"
        );
    }
}
