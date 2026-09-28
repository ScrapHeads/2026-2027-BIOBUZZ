package org.firstinspires.ftc.teamcode.robotTests;

import static org.firstinspires.ftc.teamcode.util.TestUtils.createMockTelemetry;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.TestUtils;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class RobotClassTest {

    public TestUtils.MockHardwareMap createMockHardwareMap () {
        TestUtils.HardwareMapBuilder mockHardwareMap = new TestUtils.HardwareMapBuilder()
                .withMotor("intake")
                .withMotor("transfer");
        return mockHardwareMap.build();
    }

    public Robot createRobot () {
        // Start mock FtcDashboard instance so FtcDashboard.getInstance() returns non-null on desktop JVM
        TestUtils.startMockDashboard();

        TestUtils.MockHardwareMap mockHardwareMap = createMockHardwareMap();
        Telemetry mockTelemetry = createMockTelemetry();
        return new Robot(mockHardwareMap, mockTelemetry);
    }

    @Test
    @DisplayName("Init hardwareMap, telemetry, stateIO, and ftcDashboard")
    public void InitHardwareMapTelemetryStateIoAndFtcDashboardTest () {
        Robot robot = createRobot();

        assertNotNull(robot.hm, "HardwareMap should not be null");
        assertNotNull(robot.telemetry, "Telemetry should not be null");
        assertNotNull(robot.dashboard, "FtcDashboard should not be null");
        assertNotNull(robot.state, "StateIO should not be null");
    }

    @Test
    @DisplayName("Assert subsystems are not null and registered")
    public void assertSubsystemsAreNotNullAndRegisteredTest () {
        Robot robot = createRobot();

        assertNotNull(robot.intake, "Intake should not be null");
        assertNotNull(robot.transfer, "Transfer should not be null");

        assertTrue(TestUtils.isSubsystemRegistered(robot.intake), "IntakeSubsystem should be registered with CommandScheduler");
        assertTrue(TestUtils.isSubsystemRegistered(robot.transfer), "Transfer subsystem should be registered with CommandScheduler");
    }
}
