package org.firstinspires.ftc.teamcode.RilLib.Control.PID;

/**
 * This class is for adding a Feedforward on top of a PID controller.
 * The plan usage for this class is for the flywheel.
 *
 * Tune kS
 * Slowly increase motor power from 0.
 * Record the smallest power where the flywheel consistently begins moving.
 * That value is roughly your kS.
 *
 * Tune kV
 * Run the flywheel at several steady powers, such as:
 * 0.2, 0.4, 0.6, 0.8
 * Record the steady RPM at each.
 * Calculate:
 * kV ≈ (motorPower - kS) / RPM
 * Average the results.
 *
 * Tune kA last
 * Usually start with:
 * kA = 0;
 * If acceleration is noticeably slower than commanded, increase kA slightly.
 * For an FTC flywheel, you may find you don't need it at all.
 */

public class SimpleMotorFeedforward {

    // power needed to overcome static friction
    private final double kS;

    // power needed for a particular velocity
    private final double kV;

    // additional power required while accelerating
    private final double kA;

    public SimpleMotorFeedforward(
            double kS,
            double kV,
            double kA
    ) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(
            double velocity,
            double acceleration
    ) {
        return kS * Math.signum(velocity)
                + kV * velocity
                + kA * acceleration;
    }
}
