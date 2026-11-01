package org.firstinspires.ftc.teamcode.pathing.paths;

// schemaVersion=3
// className=RedPath
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
// robotLen=17.5
// robotWid=16
// dataMeasurementUnit=in
// dataOrigin=bottomLeft
// dataCoordinateSystem=pedro
import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class RedPath {
    // plannerSegment={"name":"MoveToShootPreLoaded","color":"#666666","method":"moveToShootPreLoaded"}
    private final PoseFactory moveToShootPreLoadedPoses = PoseFactory.degrees();
    public final Pose moveToShootPreLoadedPoint1 = moveToShootPreLoadedPoses.of(59.156462585, 9.149659864, 90.000000000); // role=anchor anchor=0 locked=false
    public final Pose moveToShootPreLoadedPoint2 = moveToShootPreLoadedPoses.of(59.156462585, 15.027210884, 90.000000000); // role=anchor anchor=1 locked=false

    public Path moveToShootPreLoaded() {
        return Paths.line(moveToShootPreLoadedPoint1, moveToShootPreLoadedPoint2).linear(moveToShootPreLoadedPoint1, moveToShootPreLoadedPoint2);
    }
    // endPlannerSegment

    // plannerSegment={"name":"PickupGarden","color":"#666666","method":"pickupGarden"}
    private final PoseFactory pickupGardenPoses = PoseFactory.degrees();
    public final Pose pickupGardenPoint1 = pickupGardenPoses.of(59.156462585, 15.027210884, 90.000000000); // role=anchor anchor=0 locked=false
    public final Pose pickupGardenPoint2 = pickupGardenPoses.of(38.258503401, 12.632653061, 180.000000000); // role=anchor anchor=1 locked=false
    public final Pose pickupGardenPoint3 = pickupGardenPoses.of(10.829931973, 9.585034014, 180.000000000); // role=anchor anchor=2 locked=false

    public Path pickupGarden() {
        return Paths.path(
                Paths.line(pickupGardenPoint1, pickupGardenPoint2).tangent(),
                Paths.line(pickupGardenPoint2, pickupGardenPoint3).constant(pickupGardenPoint2)
        );
    }
    // endPlannerSegment

    // plannerSegment={"name":"LaunchGarden","color":"#666666","method":"launchGarden"}
    private final PoseFactory launchGardenPoses = PoseFactory.degrees();
    public final Pose launchGardenPoint1 = launchGardenPoses.of(10.394557823, 8.931972789, 180.000000000); // role=anchor anchor=0 locked=false
    public final Pose launchGardenPoint2 = launchGardenPoses.of(59.156462585, 15.027210884, 90.000000000); // role=anchor anchor=1 locked=false
    private final Pose launchGardenControl2_1 = launchGardenPoses.of(33.034013605, 16.986394558, 0.000000000); // role=control anchor=1 control=0 locked=false

    public Path launchGarden() {
        return Paths.curve(launchGardenPoint1, launchGardenControl2_1, launchGardenPoint2).linear(launchGardenPoint1, launchGardenPoint2);
    }
    // endPlannerSegment

    // plannerSegment={"name":"PickUpLeftFlower","color":"#666666","method":"pickUpLeftFlower"}
    private final PoseFactory pickUpLeftFlowerPoses = PoseFactory.degrees();
    public final Pose pickUpLeftFlowerPoint1 = pickUpLeftFlowerPoses.of(59.156462585, 15.027210884, 90.000000000); // role=anchor anchor=0 locked=false
    public final Pose pickUpLeftFlowerPoint2 = pickUpLeftFlowerPoses.of(14.748299320, 53.557823129, 180.000000000); // role=anchor anchor=1 locked=false
    private final Pose pickUpLeftFlowerControl2_1 = pickUpLeftFlowerPoses.of(13.659863946, 27.870748299, 0.000000000); // role=control anchor=1 control=0 locked=false

    public Path pickUpLeftFlower() {
        return Paths.curve(pickUpLeftFlowerPoint1, pickUpLeftFlowerControl2_1, pickUpLeftFlowerPoint2).linear(pickUpLeftFlowerPoint1, pickUpLeftFlowerPoint2);
    }
    // endPlannerSegment

    // plannerSegment={"name":"LaunchLeftFlower","color":"#666666","method":"launchLeftFlower"}
    private final PoseFactory launchLeftFlowerPoses = PoseFactory.degrees();
    public final Pose launchLeftFlowerPoint1 = launchLeftFlowerPoses.of(14.748299320, 53.557823129, 180.000000000); // role=anchor anchor=0 locked=false
    public final Pose launchLeftFlowerPoint2 = launchLeftFlowerPoses.of(59.809523810, 126.265306122, 270.000000000); // role=anchor anchor=1 locked=false
    private final Pose launchLeftFlowerControl2_1 = launchLeftFlowerPoses.of(24.761904762, 108.414965986, 0.000000000); // role=control anchor=1 control=0 locked=false

    public Path launchLeftFlower() {
        return Paths.curve(launchLeftFlowerPoint1, launchLeftFlowerControl2_1, launchLeftFlowerPoint2).linear(launchLeftFlowerPoint1, launchLeftFlowerPoint2);
    }
    // endPlannerSegment

    // plannerSegment={"name":"PickUpBackFlower","color":"#666666","method":"pickUpBackFlower"}
    private final PoseFactory pickUpBackFlowerPoses = PoseFactory.degrees();
    public final Pose pickUpBackFlowerPoint1 = pickUpBackFlowerPoses.of(59.809523810, 126.265306122, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose pickUpBackFlowerPoint2 = pickUpBackFlowerPoses.of(28.462585034, 121.693877551, 90.000000000); // role=anchor anchor=1 locked=false
    private final Pose pickUpBackFlowerControl2_1 = pickUpBackFlowerPoses.of(44.789115646, 116.034013605, 0.000000000); // role=control anchor=1 control=0 locked=false
    public final Pose pickUpBackFlowerPoint3 = pickUpBackFlowerPoses.of(53.714285714, 128.659863946, 90.000000000); // role=anchor anchor=2 locked=false

    public Path pickUpBackFlower() {
        return Paths.path(
                Paths.curve(pickUpBackFlowerPoint1, pickUpBackFlowerControl2_1, pickUpBackFlowerPoint2).linear(pickUpBackFlowerPoint1, pickUpBackFlowerPoint2),
                Paths.line(pickUpBackFlowerPoint2, pickUpBackFlowerPoint3).linear(pickUpBackFlowerPoint2, pickUpBackFlowerPoint3)
        );
    }
    // endPlannerSegment

    // plannerSegment={"name":"LaunchBackFlower","color":"#666666","method":"launchBackFlower"}
    private final PoseFactory launchBackFlowerPoses = PoseFactory.degrees();
    public final Pose launchBackFlowerPoint1 = launchBackFlowerPoses.of(53.714285714, 128.659863946, 90.000000000); // role=anchor anchor=0 locked=false
    public final Pose launchBackFlowerPoint2 = launchBackFlowerPoses.of(59.809523810, 126.047619048, 270.000000000); // role=anchor anchor=1 locked=false

    public Path launchBackFlower() {
        return Paths.line(launchBackFlowerPoint1, launchBackFlowerPoint2).constant(launchBackFlowerPoint2);
    }
    // endPlannerSegment

    // plannerSegment={"name":"Park","color":"#666666","method":"park"}
    private final PoseFactory parkPoses = PoseFactory.degrees();
    public final Pose parkPoint1 = parkPoses.of(59.809523810, 126.047619048, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose parkPoint2 = parkPoses.of(16.054421769, 124.088435374, 180.000000000); // role=anchor anchor=1 locked=false

    public Path park() {
        return Paths.line(parkPoint1, parkPoint2).linear(parkPoint1, parkPoint2);
    }
    // endPlannerSegment
}
