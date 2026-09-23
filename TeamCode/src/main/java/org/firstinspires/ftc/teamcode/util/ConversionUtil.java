package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.MathUtil;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;

public class ConversionUtil {

    /**
     * Converts a RilLib Pose2d (meters) to a Pedro Pose (inches).
     *
     * @param rilPose The RilLib Pose2d to convert.
     * @return The converted Pedro Pose.
     */
    public static com.pedropathing.math.Pose toPedro(Pose2d rilPose) {
        return new com.pedropathing.math.Pose(
                Units.metersToInches(rilPose.getX()),
                Units.metersToInches(rilPose.getY()),
                rilPose.getRotation().getRadians()
        );
    }

    /**
     * Converts a Pedro Pose (inches) to a RilLib Pose2d (meters).
     *
     * @param pedroPose The Pedro Pose to convert.
     * @return The converted RilLib Pose2d.
     */
    public static Pose2d toRil(Pose pedroPose) {
        return new Pose2d(
                Units.inchesToMeters(pedroPose.getX()),
                Units.inchesToMeters(pedroPose.getY()),
                new Rotation2d(pedroPose.getHeading())
        );
    }

    /**
     * Converts RilLib ChassisSpeeds (m/s) to a Pedro Pose (inches/s) used as velocity.
     *
     * @param speeds The RilLib ChassisSpeeds to convert.
     * @return The converted Pedro Pose (velocity).
     */
    public static Pose toPedro(ChassisSpeeds speeds) {
        return new Pose(
                Units.metersToInches(speeds.vxMetersPerSecond),
                Units.metersToInches(speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond
        );
    }

    /**
     * Converts a Pedro Pose (inches/s) used as velocity to RilLib ChassisSpeeds (m/s).
     *
     * @param pedroVelocity The Pedro Pose (velocity) to convert.
     * @return The converted RilLib ChassisSpeeds.
     */
    public static ChassisSpeeds toRilSpeeds(Pose pedroVelocity) {
        return new ChassisSpeeds(
                Units.inchesToMeters(pedroVelocity.getX()),
                Units.inchesToMeters(pedroVelocity.getY()),
                pedroVelocity.getHeading()
        );
    }

    public static double wrapAngleDeg(double angle) {
        return MathUtil.inputModulus(angle, -180, 180);
    }
}
