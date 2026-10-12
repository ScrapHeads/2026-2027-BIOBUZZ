package org.firstinspires.ftc.teamcode.vision.tracking;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class LimelightOutputReader {

    public static double[] getFullPythonOutput(LLResult result, MultipleTelemetry telemetry) {
        if (result == null) {
            telemetry.addLine("Results Null");
            return new double[0];
        }

        try {
            JSONObject json = new JSONObject(result.toString());
            JSONArray array = json.optJSONArray("PythonOut");

            if (array == null) {
                telemetry.addLine("Array Null");
                return new double[0];
            }

            telemetry.addData("Array length", array.length());

            double[] output = new double[array.length()];

            for (int i = 0; i < array.length(); i++) {
                output[i] = array.optDouble(i);
            }

            return output;

        } catch (JSONException e) {
            return new double[0];
        }
    }
}