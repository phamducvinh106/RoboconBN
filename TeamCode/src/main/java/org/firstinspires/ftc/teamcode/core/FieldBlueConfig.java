package org.firstinspires.ftc.teamcode.core;

import java.util.LinkedHashMap;
import java.util.Map;

/** Field coordinates for blue alliance; red mirrors factory slot order top↔bottom. */
public final class FieldBlueConfig {
    private static final String[] BLOCK_CODES = {"01", "02", "03", "04"};

    public static final class Slot {
        public final LiftingSequenceConfig.Pose near;
        public final LiftingSequenceConfig.Pose placement;

        public Slot(LiftingSequenceConfig.Pose near, LiftingSequenceConfig.Pose placement) {
            this.near = near;
            this.placement = placement;
        }
    }

    public final int version;
    public final boolean calibrated;
    public final LiftingSequenceConfig.Pose placeAtFactory;
    public final LiftingSequenceConfig.Pose shelfApproach;
    public final LiftingSequenceConfig.Pose retreat;
    public final LiftingSequenceConfig.Pose[] shelfFacs;
    public final Slot[] factorySlots;

    FieldBlueConfig(int version, boolean calibrated, LiftingSequenceConfig.Pose placeAtFactory,
                    LiftingSequenceConfig.Pose shelfApproach, LiftingSequenceConfig.Pose retreat,
                    LiftingSequenceConfig.Pose[] shelfFacs, Slot[] factorySlots) {
        this.version = version;
        this.calibrated = calibrated;
        this.placeAtFactory = placeAtFactory;
        this.shelfApproach = shelfApproach;
        this.retreat = retreat;
        this.shelfFacs = shelfFacs;
        this.factorySlots = factorySlots;
    }

    public Map<String, LiftingSequenceConfig.Factory> factoriesFor(Alliance alliance) {
        Map<String, LiftingSequenceConfig.Factory> map = new LinkedHashMap<>();
        for (int i = 0; i < BLOCK_CODES.length; i++) {
            int slotIndex = alliance == Alliance.BLUE ? i : BLOCK_CODES.length - 1 - i;
            Slot slot = factorySlots[slotIndex];
            String code = BLOCK_CODES[i];
            map.put(code, new LiftingSequenceConfig.Factory(code, slot.near, slot.placement));
        }
        return map;
    }
}
