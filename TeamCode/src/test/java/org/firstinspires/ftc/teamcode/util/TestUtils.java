package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

/**
 * Common testing utilities for FTC unit tests.
 * Provides mock implementations for HardwareMap, DcMotorEx, Servo, Telemetry,
 * and a generic createSubsystem method to dynamically instantiate any subsystem.
 */
public class TestUtils {

    /**
     * Reusable MockHardwareMap subclass that bypasses Android native library lookups
     * during host JVM unit testing.
     */
    public static class MockHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        public MockHardwareMap() {
            super(null, null);
        }

        public void putDevice(String name, HardwareDevice device) {
            devices.put(name, device);
            put(name, device);
            if (device instanceof DcMotorEx) {
                dcMotor.put(name, (DcMotorEx) device);
            } else if (device instanceof Servo) {
                servo.put(name, (Servo) device);
            }
        }

        @Override
        @SuppressWarnings("unchecked")
        public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
            HardwareDevice device = devices.get(deviceName);
            if (device != null && classOrInterface.isInstance(device)) {
                return (T) device;
            }
            throw new IllegalArgumentException("Device not found in MockHardwareMap: " + deviceName);
        }
    }

    /**
     * Fluent Builder for creating customized MockHardwareMaps with arbitrary device names.
     */
    public static class HardwareMapBuilder {
        private final MockHardwareMap map = new MockHardwareMap();

        public HardwareMapBuilder withMotor(String name) {
            map.putDevice(name, createMockMotor());
            return this;
        }

        public HardwareMapBuilder withServo(String name) {
            map.putDevice(name, createMockServo());
            return this;
        }

        public HardwareMapBuilder withDevice(String name, HardwareDevice device) {
            map.putDevice(name, device);
            return this;
        }

        public MockHardwareMap build() {
            return map;
        }
    }

    /**
     * Creates a mock DcMotorEx proxy that tracks setPower/getPower and provides valid configuration types.
     */
    public static DcMotorEx createMockMotor() {
        final double[] currentPower = {0.0};
        MotorConfigurationType dummyType = new MotorConfigurationType();

        return (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower":
                            if (args != null && args.length > 0) {
                                currentPower[0] = (Double) args[0];
                            }
                            return null;
                        case "getPower":
                            return currentPower[0];
                        case "getMotorType":
                            return dummyType;
                        case "hashCode":
                            return System.identityHashCode(proxy);
                        case "equals":
                            return args != null && args.length > 0 && proxy == args[0];
                        case "toString":
                            return "MockDcMotorEx";
                        default:
                            Class<?> returnType = method.getReturnType();
                            if (returnType == boolean.class) return Boolean.FALSE;
                            if (returnType == int.class) return 0;
                            if (returnType == double.class) return 0.0;
                            return null;
                    }
                }
        );
    }

    /**
     * Creates a mock Servo proxy that tracks position and direction.
     */
    public static Servo createMockServo() {
        final double[] currentPosition = {0.0};
        final Servo.Direction[] currentDirection = {Servo.Direction.FORWARD};

        return (Servo) Proxy.newProxyInstance(
                Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPosition":
                            if (args != null && args.length > 0) {
                                currentPosition[0] = (Double) args[0];
                            }
                            return null;
                        case "getPosition":
                            return currentPosition[0];
                        case "setDirection":
                            if (args != null && args.length > 0) {
                                currentDirection[0] = (Servo.Direction) args[0];
                            }
                            return null;
                        case "getDirection":
                            return currentDirection[0];
                        case "hashCode":
                            return System.identityHashCode(proxy);
                        case "equals":
                            return args != null && args.length > 0 && proxy == args[0];
                        case "toString":
                            return "MockServo";
                        default:
                            Class<?> returnType = method.getReturnType();
                            if (returnType == boolean.class) return Boolean.FALSE;
                            if (returnType == int.class) return 0;
                            if (returnType == double.class) return 0.0;
                            return null;
                    }
                }
        );
    }

    /**
     * Creates a mock Telemetry proxy.
     */
    public static Telemetry createMockTelemetry() {
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
                            return "MockTelemetry";
                        default:
                            Class<?> returnType = method.getReturnType();
                            if (returnType == boolean.class) return Boolean.FALSE;
                            if (returnType == int.class) return 0;
                            if (returnType == double.class) return 0.0;
                            return null;
                    }
                }
        );
    }

    /**
     * Initializes FtcDashboard.getInstance() on desktop JVM unit tests without triggering Android native library lookups.
     * Uses Reflection and Unsafe to instantiate FtcDashboard without calling native hardware/network constructors,
     * and sets the static instance field.
     */
    public static com.acmerobotics.dashboard.FtcDashboard startMockDashboard() {
        try {
            Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
            java.lang.reflect.Field unsafeField = unsafeClass.getDeclaredField("theUnsafe");
            unsafeField.setAccessible(true);
            Object unsafe = unsafeField.get(null);

            java.lang.reflect.Method allocateInstance = unsafeClass.getMethod("allocateInstance", Class.class);
            com.acmerobotics.dashboard.FtcDashboard mockDashboard =
                    (com.acmerobotics.dashboard.FtcDashboard) allocateInstance.invoke(unsafe, com.acmerobotics.dashboard.FtcDashboard.class);

            java.lang.reflect.Field instanceField = com.acmerobotics.dashboard.FtcDashboard.class.getDeclaredField("instance");
            instanceField.setAccessible(true);
            instanceField.set(null, mockDashboard);

            return mockDashboard;
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize mock FtcDashboard", e);
        }
    }

    /**
     * Creates a MockHardwareMap populated with mock motors for all specified device names.
     */
    public static MockHardwareMap createHardwareMapWithMotors(String... motorNames) {
        HardwareMapBuilder builder = new HardwareMapBuilder();
        for (String name : motorNames) {
            builder.withMotor(name);
        }
        return builder.build();
    }

    /**
     * Creates a MockHardwareMap populated with mock servos for all specified device names.
     */
    public static MockHardwareMap createHardwareMapWithServos(String... servoNames) {
        HardwareMapBuilder builder = new HardwareMapBuilder();
        for (String name : servoNames) {
            builder.withServo(name);
        }
        return builder.build();
    }

    //----------------------------------------------------------------------------------------------
    // Generic Single-Method Subsystem Creation
    //----------------------------------------------------------------------------------------------

    /**
     * Universal single method to instantiate and register ANY subsystem using its constructor reference
     * (e.g. IntakeSubsystem::new, Transfer::new) and a list of required motor device names.
     *
     * Example Usage:
     *   IntakeSubsystem intake = TestUtils.createSubsystem(IntakeSubsystem::new, "intake");
     *   Transfer transfer = TestUtils.createSubsystem(Transfer::new, "transfer");
     *
     * @param constructor Subsystem constructor reference accepting HardwareMap (e.g. MySubsystem::new)
     * @param requiredMotorNames Device names of motors to populate in the mock HardwareMap
     * @param <T> Subsystem type
     * @return The instantiated and registered subsystem
     */
    public static <T extends Subsystem> T createSubsystem(
            Function<HardwareMap, T> constructor,
            String... requiredMotorNames) {
        MockHardwareMap hardwareMap = createHardwareMapWithMotors(requiredMotorNames);
        T subsystem = constructor.apply(hardwareMap);
        subsystem.register();
        return subsystem;
    }

    /**
     * Universal single method to instantiate and register ANY subsystem using its constructor reference
     * and a pre-built HardwareMap (e.g. built via HardwareMapBuilder).
     *
     * Example Usage:
     *   ArmSubsystem arm = TestUtils.createSubsystem(ArmSubsystem::new, myHardwareMap);
     *
     * @param constructor Subsystem constructor reference accepting HardwareMap (e.g. MySubsystem::new)
     * @param hardwareMap Pre-configured HardwareMap
     * @param <T> Subsystem type
     * @return The instantiated and registered subsystem
     */
    public static <T extends Subsystem> T createSubsystem(
            Function<HardwareMap, T> constructor,
            HardwareMap hardwareMap) {
        T subsystem = constructor.apply(hardwareMap);
        subsystem.register();
        return subsystem;
    }

    // Convenience methods retained for backwards compatibility
    public static IntakeSubsystem createDummyIntakeSubsystem() {
        return createSubsystem(IntakeSubsystem::new, "intake");
    }

    public static TransferSubsystem createDummyTransferSubsystem() {
        return createSubsystem(TransferSubsystem::new, "transfer");
    }

    /**
     * Checks if a given subsystem is currently registered with the SolversLib CommandScheduler.
     */
    public static boolean isSubsystemRegistered(Subsystem subsystem) {
        try {
            java.lang.reflect.Field field = com.seattlesolvers.solverslib.command.CommandScheduler.class.getDeclaredField("m_subsystems");
            field.setAccessible(true);
            java.util.Map<?, ?> map = (java.util.Map<?, ?>) field.get(com.seattlesolvers.solverslib.command.CommandScheduler.getInstance());
            return map.containsKey(subsystem);
        } catch (Exception e) {
            return false;
        }
    }
}
