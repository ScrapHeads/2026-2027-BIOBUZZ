package org.firstinspires.ftc.teamcode.vision.tracking;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.RilLib.Math.MathUtil;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;

import java.util.ArrayList;
import java.util.List;

public class GameElementMap {
    /**
     * Owns all currently tracked elements,
     * potentially up to the full 56.
     */
    private final List<TrackedGameElement> gameElementMap;

    public GameElementMap () {
        this.gameElementMap = new ArrayList<TrackedGameElement>();
    }

    public GameElementMap (List<TrackedGameElement> gameElementMap) {
        this.gameElementMap = gameElementMap;
    }

    public List<TrackedGameElement> getGameElementMap () {
        return gameElementMap;
    }

    public void addElement (TrackedGameElement trackedGameElement) {
        if (gameElementMap.stream().noneMatch(gameElementMap ->
                gameElementMap.getId() == trackedGameElement.getId())) {
            gameElementMap.add(trackedGameElement);
        }
    }

    public void removeElement (TrackedGameElement trackedGameElement) {
        gameElementMap.remove(trackedGameElement);
    }

    public int getSize () {
        return gameElementMap.size();
    }

    public String returnAllIds () {
        if (gameElementMap.isEmpty()) {
            return "";
        }
        StringBuilder outputId = new StringBuilder();
        for (int i = 0; i < gameElementMap.size(); i++) {
            outputId.append(gameElementMap.get(i).getId()).append(" ");
        }
        return outputId.toString();
    }

    public String returnAmountOfEachElement () {
        if (gameElementMap.isEmpty()) {
            return "";
        }
        int pollenAmount = 0;
        int nectarBlueAmount = 0;
        int nectarRedAmount = 0;

        for (TrackedGameElement element : gameElementMap) {
            switch (element.getType()) {
                case POLLEN:
                    pollenAmount++;
                    break;

                case BLUE_NECTAR:
                    nectarBlueAmount++;
                    break;

                case RED_NECTAR:
                    nectarRedAmount++;
                    break;

                default:
                    break;
            }
        }
        return "Pollen: " + pollenAmount + "; Blue Nectar: " + nectarBlueAmount + "; Red Nectar: " + nectarRedAmount;
    }

    public void drawMap (TelemetryPacket packet) {
        Canvas c = packet.fieldOverlay();
        c.setTranslation(-72.0, -72.0);
        for (TrackedGameElement element : gameElementMap) {
            switch (element.getType()) {
                case POLLEN:
                    GameElementDrawing.drawPollen(c, ConversionUtil.toPedro(element.getEstimatedPose()));
                    break;

                case BLUE_NECTAR:
                    GameElementDrawing.drawBlueNectar(c, ConversionUtil.toPedro(element.getEstimatedPose()));
                    break;

                case RED_NECTAR:
                    GameElementDrawing.drawRedNectar(c, ConversionUtil.toPedro(element.getEstimatedPose()));
                    break;

                default:
                    break;
            }
        }
    }
}
