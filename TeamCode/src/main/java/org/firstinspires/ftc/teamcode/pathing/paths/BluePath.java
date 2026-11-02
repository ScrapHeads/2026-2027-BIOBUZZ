package org.firstinspires.ftc.teamcode.pathing.paths;

// schemaVersion=3
// className=BluePath
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

public final class BluePath {
    // plannerSegment={"name":"MoveToShootPreLoaded","color":"#666666","method":"moveToShootPreLoaded"}
    private final PoseFactory moveToShootPreLoadedPoses = PoseFactory.degrees();
    public final Pose moveToShootPreLoadedPoint1 = moveToShootPreLoadedPoses.of(84.843537415, 134.850340136, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose moveToShootPreLoadedPoint2 = moveToShootPreLoadedPoses.of(85.065539062, 130.119663543, 270.000000000); // role=anchor anchor=1 locked=false
    
    public Path moveToShootPreLoaded() {
        return Paths.line(moveToShootPreLoadedPoint1, moveToShootPreLoadedPoint2).linear(moveToShootPreLoadedPoint1, moveToShootPreLoadedPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"PrePickupGarden","color":"#666666","method":"prePickupGarden"}
    private final PoseFactory prePickupGardenPoses = PoseFactory.degrees();
    public final Pose prePickupGardenPoint1 = prePickupGardenPoses.of(85.065539062, 130.119663543, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose prePickupGardenPoint2 = prePickupGardenPoses.of(106.445512072, 132.677950911, 0.000000000); // role=anchor anchor=1 locked=false
    
    public Path prePickupGarden() {
        return Paths.line(prePickupGardenPoint1, prePickupGardenPoint2).linear(prePickupGardenPoint1, prePickupGardenPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"PickUpGarden","color":"#666666","method":"pickUpGarden"}
    private final PoseFactory pickUpGardenPoses = PoseFactory.degrees();
    public final Pose pickUpGardenPoint1 = pickUpGardenPoses.of(106.445512072, 132.677950911, 0.000000000); // role=anchor anchor=0 locked=false
    public final Pose pickUpGardenPoint2 = pickUpGardenPoses.of(133.307529444, 134.505299032, 5.000000000); // role=anchor anchor=1 locked=false
    private final Pose pickUpGardenControl2_1 = pickUpGardenPoses.of(119.785153352, 135.236238280, 0.000000000); // role=control anchor=1 control=0 locked=false
    
    public Path pickUpGarden() {
        return Paths.curve(pickUpGardenPoint1, pickUpGardenControl2_1, pickUpGardenPoint2).constant(pickUpGardenPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"LaunchGarden","color":"#666666","method":"launchGarden"}
    private final PoseFactory launchGardenPoses = PoseFactory.degrees();
    public final Pose launchGardenPoint1 = launchGardenPoses.of(133.307529444, 134.505299032, 5.000000000); // role=anchor anchor=0 locked=false
    public final Pose launchGardenPoint2 = launchGardenPoses.of(84.882804250, 130.119663543, 270.000000000); // role=anchor anchor=1 locked=false
    private final Pose launchGardenControl2_1 = launchGardenPoses.of(111.013882373, 128.292315422, 0.000000000); // role=control anchor=1 control=0 locked=false
    
    public Path launchGarden() {
        return Paths.curve(launchGardenPoint1, launchGardenControl2_1, launchGardenPoint2).linear(launchGardenPoint1, launchGardenPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"PrePickUpLeftFlower","color":"#666666","method":"prePickUpLeftFlower"}
    private final PoseFactory prePickUpLeftFlowerPoses = PoseFactory.degrees();
    public final Pose prePickUpLeftFlowerPoint1 = prePickUpLeftFlowerPoses.of(84.882804250, 130.119663543, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose prePickUpLeftFlowerPoint2 = prePickUpLeftFlowerPoses.of(109.917473501, 92.293557448, 0.000000000); // role=anchor anchor=1 locked=false
    private final Pose prePickUpLeftFlowerControl2_1 = prePickUpLeftFlowerPoses.of(112.475760870, 117.510961511, 0.000000000); // role=control anchor=1 control=0 locked=false
    
    public Path prePickUpLeftFlower() {
        return Paths.curve(prePickUpLeftFlowerPoint1, prePickUpLeftFlowerControl2_1, prePickUpLeftFlowerPoint2).linear(prePickUpLeftFlowerPoint1, prePickUpLeftFlowerPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"PickUpLeftFlower","color":"#666666","method":"pickUpLeftFlower"}
    private final PoseFactory pickUpLeftFlowerPoses = PoseFactory.degrees();
    public final Pose pickUpLeftFlowerPoint1 = pickUpLeftFlowerPoses.of(109.917473501, 92.293557448, 0.000000000); // role=anchor anchor=0 locked=false
    public final Pose pickUpLeftFlowerPoint2 = pickUpLeftFlowerPoses.of(129.652833203, 92.110822636, 0.000000000); // role=anchor anchor=1 locked=false
    
    public Path pickUpLeftFlower() {
        return Paths.line(pickUpLeftFlowerPoint1, pickUpLeftFlowerPoint2).constant(pickUpLeftFlowerPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"LaunchLeftFlower","color":"#666666","method":"launchLeftFlower"}
    private final PoseFactory launchLeftFlowerPoses = PoseFactory.degrees();
    public final Pose launchLeftFlowerPoint1 = launchLeftFlowerPoses.of(129.652833203, 92.110822636, 0.000000000); // role=anchor anchor=0 locked=false
    public final Pose launchLeftFlowerPoint2 = launchLeftFlowerPoses.of(109.003799441, 57.939412782, 90.000000000); // role=anchor anchor=1 locked=false
    private final Pose launchLeftFlowerControl2_1 = launchLeftFlowerPoses.of(109.003799441, 67.258888197, 0.000000000); // role=control anchor=1 control=0 locked=false
    public final Pose launchLeftFlowerPoint3 = launchLeftFlowerPoses.of(84.334599814, 11.870253525, 90.000000000); // role=anchor anchor=2 locked=false
    private final Pose launchLeftFlowerControl3_1 = launchLeftFlowerPoses.of(110.282943125, 37.838583456, 0.000000000); // role=control anchor=2 control=0 locked=false
    
    public Path launchLeftFlower() {
        return Paths.path(
            Paths.curve(launchLeftFlowerPoint1, launchLeftFlowerControl2_1, launchLeftFlowerPoint2).linear(launchLeftFlowerPoint1, launchLeftFlowerPoint2),
            Paths.curve(launchLeftFlowerPoint2, launchLeftFlowerControl3_1, launchLeftFlowerPoint3).linear(launchLeftFlowerPoint2, launchLeftFlowerPoint3)
        );
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"PrePickUpBackFlower","color":"#666666","method":"prePickUpBackFlower"}
    private final PoseFactory prePickUpBackFlowerPoses = PoseFactory.degrees();
    public final Pose prePickUpBackFlowerPoint1 = prePickUpBackFlowerPoses.of(84.334599814, 11.870253525, 90.000000000); // role=anchor anchor=0 locked=false
    public final Pose prePickUpBackFlowerPoint2 = prePickUpBackFlowerPoses.of(92.374931544, 26.509025109, 270.000000000); // role=anchor anchor=1 locked=false
    
    public Path prePickUpBackFlower() {
        return Paths.line(prePickUpBackFlowerPoint1, prePickUpBackFlowerPoint2).linear(prePickUpBackFlowerPoint1, prePickUpBackFlowerPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"PickUpBackFlower","color":"#666666","method":"pickUpBackFlower"}
    private final PoseFactory pickUpBackFlowerPoses = PoseFactory.degrees();
    public final Pose pickUpBackFlowerPoint1 = pickUpBackFlowerPoses.of(92.374931544, 26.509025109, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose pickUpBackFlowerPoint2 = pickUpBackFlowerPoses.of(92.557666356, 15.910406010, 270.000000000); // role=anchor anchor=1 locked=false
    
    public Path pickUpBackFlower() {
        return Paths.line(pickUpBackFlowerPoint1, pickUpBackFlowerPoint2).constant(pickUpBackFlowerPoint2);
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"LaunchBackFlower","color":"#666666","method":"launchBackFlower"}
    private final PoseFactory launchBackFlowerPoses = PoseFactory.degrees();
    public final Pose launchBackFlowerPoint1 = launchBackFlowerPoses.of(92.557666356, 15.910406010, 270.000000000); // role=anchor anchor=0 locked=false
    public final Pose launchBackFlowerPoint2 = launchBackFlowerPoses.of(92.740401168, 26.326290297, 270.000000000); // role=anchor anchor=1 locked=false
    public final Pose launchBackFlowerPoint3 = launchBackFlowerPoses.of(85.065539062, 14.428540893, 90.000000000); // role=anchor anchor=2 locked=false
    
    public Path launchBackFlower() {
        return Paths.path(
            Paths.line(launchBackFlowerPoint1, launchBackFlowerPoint2).constant(launchBackFlowerPoint2),
            Paths.line(launchBackFlowerPoint2, launchBackFlowerPoint3).linear(launchBackFlowerPoint2, launchBackFlowerPoint3)
        );
    }
    // endPlannerSegment
    
    // plannerSegment={"name":"Park","color":"#666666","method":"park"}
    private final PoseFactory parkPoses = PoseFactory.degrees();
    public final Pose parkPoint1 = parkPoses.of(85.065539062, 14.428540893, 90.000000000); // role=anchor anchor=0 locked=false
    public final Pose parkPoint2 = parkPoses.of(130.566507263, 21.209715559, 90.000000000); // role=anchor anchor=1 locked=false
    private final Pose parkControl2_1 = parkPoses.of(97.491506281, 25.778085861, 0.000000000); // role=control anchor=1 control=0 locked=false
    
    public Path park() {
        return Paths.curve(parkPoint1, parkControl2_1, parkPoint2).linear(parkPoint1, parkPoint2);
    }
    // endPlannerSegment
}
