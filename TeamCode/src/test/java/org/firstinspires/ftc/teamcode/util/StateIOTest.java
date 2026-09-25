package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RobotState;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Proxy;

public class StateIOTest {

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    private Telemetry createDummyTelemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "hashCode":
                            return System.identityHashCode(proxy);
                        case "equals":
                            return args != null && args.length > 0 && proxy == args[0];
                        case "toString":
                            return "DummyTelemetry";
                        default:
                            Class<?> returnType = method.getReturnType();
                            if (returnType == boolean.class) {
                                return Boolean.FALSE;
                            }
                            if (returnType == int.class) {
                                return 0;
                            }
                            if (returnType == double.class) {
                                return 0.0;
                            }
                            return null;
                    }
                }
        );
    }

    @Test
    @DisplayName("Test StateIO instantiation and host JVM native library behavior")
    public void testStateIOLoadOnDesktopJVM() {
        Telemetry dummyTelemetry = createDummyTelemetry();
        StateIO stateIO = new StateIO(dummyTelemetry, null);

        assertNotNull(stateIO);
        // AppUtil requires native RobotCore.so which is unavailable on host JVM during desktop unit tests.
        // Calling load() triggers AppUtil static initialization which throws LinkageError/UnsatisfiedLinkError on desktop.
        assertThrows(Throwable.class, stateIO::load);
    }

    @Test
    @DisplayName("Test RobotState setAll and field accessors")
    public void testRobotStateSetAll() {
        RobotState state = RobotState.getInstance();

        Pose2d pose = new Pose2d(1.5, 2.5, new Rotation2d(0.5));
        ChassisSpeeds speeds = new ChassisSpeeds(0.5, -0.2, 0.1);

        state.setAll(pose, true, speeds);

        assertEquals(pose, state.getOdometryPose());
        assertEquals(true, state.isBlueTeam());
        assertEquals(speeds, state.getChassisSpeeds());
    }

    @Test
    @DisplayName("Test RobotState JSON serialization and deserialization used by StateIO")
    public void testRobotStateJsonSerialization() {
        RobotState originalState = RobotState.getInstance();
        Pose2d pose = new Pose2d(2.0, 3.0, new Rotation2d(1.0));
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.5, -0.2);

        originalState.setAll(pose, false, speeds);

        String json = GSON.toJson(originalState);
        assertNotNull(json);

        RobotState deserialized = GSON.fromJson(json, RobotState.class);
        assertNotNull(deserialized);
        assertEquals(2.0, deserialized.getOdometryPose().getX(), 1e-6);
        assertEquals(3.0, deserialized.getOdometryPose().getY(), 1e-6);
        assertEquals(false, deserialized.isBlueTeam());
        assertEquals(1.0, deserialized.getChassisSpeeds().vxMetersPerSecond, 1e-6);
    }
}
