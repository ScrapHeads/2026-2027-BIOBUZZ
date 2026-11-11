package org.firstinspires.ftc.teamcode.util.aim;

import org.firstinspires.ftc.teamcode.RilLib.Math.Units;

/** One immutable aiming snapshot shared by independently running subsystems. */
public final class AimSolution {
    public final HiveAim.Cell cell;
    public final HiveAim.Solution solution;
    public final double robotXInches, robotYInches;
    /** Distance from robot center to the solved field point, before shooter offsets. */
    public final double distanceCm;

    public AimSolution(HiveAim.Cell cell, HiveAim.Solution solution,
                       double robotXInches, double robotYInches) {
        this.cell = cell;
        this.solution = solution;
        this.robotXInches = robotXInches;
        this.robotYInches = robotYInches;
        distanceCm = 100.0 * Units.inchesToMeters(
                Math.hypot(solution.aimX - robotXInches, solution.aimY - robotYInches));
    }
}
