package org.firstinspires.ftc.teamcode.pathing.paths;

// schemaVersion=3
// className=RedTestPath
// library=pedro
// format=posefactory
// headingWrapHalf=false
// presetId=pedro-biobuzz
// presetField=ReferenceImages/biobuzzPedropathing.webp
// measurementUnit=in
// fieldSize=144
// fieldWidth=144
// fieldHeight=144
// origin=bottomLeft
// coordinateSystem=pedro
// robotLen=18
// robotWid=18
// dataMeasurementUnit=in
// dataOrigin=bottomLeft
// dataCoordinateSystem=pedro
import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class RedTestPath {
    // plannerSegment={"name":"Start","color":"#666666","method":"start"}
    private final PoseFactory startPoses = PoseFactory.degrees();
    public final Pose startPoint1 = startPoses.of(57.157699443, 9.533130135, 0.000000000); // role=anchor anchor=0 locked=false
    public final Pose startPoint2 = startPoses.of(27.809523810, 56.170068027, 0.000000000); // role=anchor anchor=1 locked=false
    private final Pose startControl2_1 = startPoses.of(14.539093560, 11.865491651, 0.000000000); // role=control anchor=1 control=0 locked=false
    public final Pose startPoint3 = startPoses.of(33.197985688, 110.884839650, 0.000000000); // role=anchor anchor=2 locked=false

    public Path start() {
        return Paths.path(
                Paths.curve(startPoint1, startControl2_1, startPoint2).linear(startPoint1, startPoint2),
                Paths.line(startPoint2, startPoint3).linear(startPoint2, startPoint3)
        );
    }
    // endPlannerSegment

    // plannerSegment={"name":"end","color":"#666666","method":"end"}
    private final PoseFactory endPoses = PoseFactory.degrees();
    public final Pose endPoint1 = endPoses.of(33.197985688, 110.884839650, 0.000000000); // role=anchor anchor=0 locked=false
    public final Pose endPoint2 = endPoses.of(91.374149660, 118.210884354, 0.000000000); // role=anchor anchor=1 locked=false
    public final Pose endPoint3 = endPoses.of(116.843537415, 69.231292517, 0.000000000); // role=anchor anchor=2 locked=false
    public final Pose endPoint4 = endPoses.of(100.081632653, 14.809523810, 0.000000000); // role=anchor anchor=3 locked=false

    public Path end() {
        return Paths.path(
                Paths.line(endPoint1, endPoint2).linear(endPoint1, endPoint2),
                Paths.line(endPoint2, endPoint3).linear(endPoint2, endPoint3),
                Paths.line(endPoint3, endPoint4).linear(endPoint3, endPoint4)
        );
    }
    // endPlannerSegment
}
