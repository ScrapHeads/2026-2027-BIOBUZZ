package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.localization.DrivetrainLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

/**
 * Pedro Pathing 3.0 constants. The values below are placeholders: run the AutoTune procedures in
 * {@link Tuning} (see https://pedropathing.com/docs/pathing/tuning) and paste the generated configs here.
 * If you use a different drivetrain or localizer, swap the config classes and {@link #createFollower} to match.
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.backLeftName.set("lb");
        c.frontRightName.set("rf");
        c.backRightName.set("rb");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.powerThreshold.set(0.01);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig pinpointLocalizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.10919853458254357);
        c.yPodOffset.set(-6.3487562044398995);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.245643156781077);
                Controller secondaryTranslationalForward = Controller.proportional(0.09075858272457327);
                Controller primaryTranslationalLateral = Controller.proportional(0.38900513632730377);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1437270034642595);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011933421629760224));
                c.brake.set(Controller.proportionalFeedforward(0.01014340838529619));

                c.headingFeedback.set(Controller.proportional(3.664578223758807));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04368933901466255, 0.0057818717651031476));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12990936041578702, 0.051654240060753996));
                c.quadraticBrakeCoefficients.set(Matrix.diag(2.48671265362582E-4, 0.0018864093977020225));

                c.maxAchievableForwardVelocity.set(80.25591394276032);
                c.maxAchievableStrafeVelocity.set(66.31193180451149);
                c.naturalForwardDeceleration.set(33.27163625305453);
                c.naturalStrafeDeceleration.set(61.632003234736956);
            }
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new DrivetrainLocalizer(hardwareMap, pinpointLocalizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
