package org.firstinspires.ftc.teamcode.vision.tracking;

import java.util.List;

public class GameElementMap {
    /**
     * Owns all currently tracked elements,
     * potentially up to the full 56.
     */
    private final List<TrackedGameElement> gameElementMap;

    public GameElementMap (List<TrackedGameElement> gameElementMap) {
        this.gameElementMap = gameElementMap;
    }

    public List<TrackedGameElement> getGameElementMap () {
        return gameElementMap;
    }

    public void addElement (TrackedGameElement trackedGameElement) {
        gameElementMap.add(trackedGameElement);
    }

    public void removeElement (TrackedGameElement trackedGameElement) {
        gameElementMap.remove(trackedGameElement);
    }
}
