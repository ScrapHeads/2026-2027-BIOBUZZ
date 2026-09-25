package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ConversionUtilTest {

    private static final double EPSILON = 1e-5;

    @Test
    @DisplayName("Test Pose2d (m) to Pedro Pose (in) conversion")
    public void testToPedroPose() {
        double xMeters = 1.0;
        double yMeters = 2.0;
        double headingRad = Math.PI / 4.0;

        Pose2d rilPose = new Pose2d(xMeters, yMeters, new Rotation2d(headingRad));
        Pose pedroPose = ConversionUtil.toPedro(rilPose);

        assertEquals(Units.metersToInches(xMeters), pedroPose.x(), EPSILON);
        assertEquals(Units.metersToInches(yMeters), pedroPose.y(), EPSILON);
        assertEquals(headingRad, pedroPose.heading(), EPSILON);
    }

    @Test
    @DisplayName("Test Pedro Pose (in) to Pose2d (m) conversion")
    public void testToRilPose() {
        double xInches = 39.3701; // ~1 meter
        double yInches = 78.7402; // ~2 meters
        double headingRad = Math.PI / 2.0;

        Pose pedroPose = new Pose(xInches, yInches, headingRad);
        Pose2d rilPose = ConversionUtil.toRil(pedroPose);

        assertEquals(Units.inchesToMeters(xInches), rilPose.getX(), EPSILON);
        assertEquals(Units.inchesToMeters(yInches), rilPose.getY(), EPSILON);
        assertEquals(headingRad, rilPose.getRotation().getRadians(), EPSILON);
    }

    @Test
    @DisplayName("Test Pose round-trip conversion (Ril -> Pedro -> Ril)")
    public void testPoseRoundTrip() {
        Pose2d originalPose = new Pose2d(1.5, -0.75, new Rotation2d(1.23));
        Pose pedroPose = ConversionUtil.toPedro(originalPose);
        Pose2d reconstructedPose = ConversionUtil.toRil(pedroPose);

        assertEquals(originalPose.getX(), reconstructedPose.getX(), EPSILON);
        assertEquals(originalPose.getY(), reconstructedPose.getY(), EPSILON);
        assertEquals(originalPose.getRotation().getRadians(), reconstructedPose.getRotation().getRadians(), EPSILON);
    }

    @Test
    @DisplayName("Test ChassisSpeeds to Pedro Pose velocity conversion")
    public void testToPedroSpeeds() {
        ChassisSpeeds speeds = new ChassisSpeeds(2.0, -1.0, 0.5);
        Pose pedroVel = ConversionUtil.toPedro(speeds);

        assertEquals(Units.metersToInches(2.0), pedroVel.x(), EPSILON);
        assertEquals(Units.metersToInches(-1.0), pedroVel.y(), EPSILON);
        assertEquals(0.5, pedroVel.heading(), EPSILON);
    }

    @Test
    @DisplayName("Test Pedro Pose velocity to ChassisSpeeds conversion")
    public void testToRilSpeeds() {
        Pose pedroVel = new Pose(39.3701, -39.3701, 1.0);
        ChassisSpeeds speeds = ConversionUtil.toRilSpeeds(pedroVel);

        assertEquals(Units.inchesToMeters(39.3701), speeds.vxMetersPerSecond, EPSILON);
        assertEquals(Units.inchesToMeters(-39.3701), speeds.vyMetersPerSecond, EPSILON);
        assertEquals(1.0, speeds.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    @DisplayName("Test angle wrapping in degrees")
    public void testWrapAngleDeg() {
        assertEquals(0.0, ConversionUtil.wrapAngleDeg(0.0), EPSILON);
        assertEquals(90.0, ConversionUtil.wrapAngleDeg(90.0), EPSILON);
        assertEquals(-90.0, ConversionUtil.wrapAngleDeg(-90.0), EPSILON);
        assertEquals(180.0, ConversionUtil.wrapAngleDeg(180.0), EPSILON);
        assertEquals(180.0, ConversionUtil.wrapAngleDeg(-180.0), EPSILON);

        // Boundary overflows
        assertEquals(-90.0, ConversionUtil.wrapAngleDeg(270.0), EPSILON);
        assertEquals(90.0, ConversionUtil.wrapAngleDeg(-270.0), EPSILON);
        assertEquals(0.0, ConversionUtil.wrapAngleDeg(360.0), EPSILON);
        assertEquals(-170.0, ConversionUtil.wrapAngleDeg(190.0), EPSILON);
        assertEquals(170.0, ConversionUtil.wrapAngleDeg(-190.0), EPSILON);
    }
}
