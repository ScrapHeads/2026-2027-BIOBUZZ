package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Arrays;
import java.util.Collections;

@TeleOp(name = "TEST - Hive FieldMap")
public class HiveFieldMapTest extends LinearOpMode {

    private static final int PIPELINE = 0;
    private static final int TEST_TAG_ID = 30;
    private static final double TAG_SIZE_MM = 82.55;

    // TEST VALUES ONLY. Replace with measured tag field transforms.
    // Row-major 4x4 matrices in meters.
    private static final double[] TAG_DOWN = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.50,
            0, 0, 0, 1
    };

    private static final double[] TAG_UP = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.80,
            0, 0, 0, 1
    };

    private Limelight3A limelight;
    private LLFieldMap upMap;
    private LLFieldMap downMap;

    private String activeMap = "NONE";
    private long uploadStartMs;
    private long lastUploadDurationMs;
    private boolean lastUploadSucceeded;

    private boolean previousA = false;
    private boolean previousB = false;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        upMap = createMap(TAG_UP);
        downMap = createMap(TAG_DOWN);

        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("A: Upload DOWN map");
        telemetry.addLine("B: Upload UP map");
        telemetry.addLine("Keep robot stationary during test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Button edge detection prevents repeated uploads.
            if (gamepad1.a && !previousA) {
                uploadMap(downMap, "DOWN");
            }

            if (gamepad1.b && !previousB) {
                uploadMap(upMap, "UP");
            }

            previousA = gamepad1.a;
            previousB = gamepad1.b;

            LLResult result = limelight.getLatestResult();

            telemetry.addData("Active Map", activeMap);
            telemetry.addData("Upload Accepted",
                    lastUploadSucceeded);
            telemetry.addData("Upload Call Duration",
                    "%d ms", lastUploadDurationMs);

            if (result != null) {
                telemetry.addData("Result Valid",
                        result.isValid());
                telemetry.addData("Result Staleness",
                        "%d ms", result.getStaleness());

                telemetry.addData("MegaTag Count",
                        result.getBotposeTagCount());

                if (result.isValid()) {

                    Pose3D pose = result.getBotpose();

                    if (pose != null) {
                        telemetry.addData(
                                "MegaTag Position", pose);
                    }

                    for (LLResultTypes.FiducialResult tag
                            : result.getFiducialResults()) {

                        telemetry.addData(
                                "Detected Tag",
                                tag.getFiducialId());

                        telemetry.addData(
                                "Tag Robot Pose",
                                tag.getTargetPoseRobotSpace());
                    }
                }
            }

            telemetry.update();
            sleep(20);
        }

        limelight.stop();
    }

    private LLFieldMap createMap(double[] transform) {

        LLFieldMap.Fiducial tag =
                new LLFieldMap.Fiducial(
                        TEST_TAG_ID,
                        TAG_SIZE_MM,
                        "apriltag3_36h11_classic",
                        Arrays.asList(
                                Arrays.stream(transform)
                                        .boxed()
                                        .toArray(Double[]::new)
                        ),
                        true
                );

        return new LLFieldMap(
                Collections.singletonList(tag),
                "ftc"
        );
    }

    private void uploadMap(
            LLFieldMap map,
            String name) {

        uploadStartMs = System.currentTimeMillis();

        lastUploadSucceeded =
                limelight.uploadFieldmap(map, PIPELINE);

        lastUploadDurationMs =
                System.currentTimeMillis() - uploadStartMs;

        if (lastUploadSucceeded) {
            activeMap = name;
        }
    }
}
