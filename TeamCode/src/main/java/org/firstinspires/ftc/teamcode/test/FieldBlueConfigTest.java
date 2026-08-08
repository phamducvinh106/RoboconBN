package org.firstinspires.ftc.teamcode.test;

import java.nio.file.Files;
import java.nio.file.Paths;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.FieldBlueConfig;
import org.firstinspires.ftc.teamcode.core.FieldBlueConfigLoader;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfigLoader;

public final class FieldBlueConfigTest {
    static int passed;
    static void check(String name, boolean ok) {
        if (!ok) throw new AssertionError(name);
        passed++;
    }

    static String fieldJson() throws Exception {
        return new String(Files.readAllBytes(Paths.get("TeamCode/src/main/assets/field-blue.json")), "UTF-8");
    }

    static LiftingSequenceConfig merged(Alliance alliance) throws Exception {
        String robot = new String(Files.readAllBytes(Paths.get("TeamCode/src/main/assets/robot-config.json")), "UTF-8");
        FieldBlueConfig field = FieldBlueConfigLoader.load(fieldJson());
        return LiftingSequenceConfigLoader.load(robot).withField(field, alliance);
    }

    public static void main(String[] args) throws Exception {
        FieldBlueConfig field = FieldBlueConfigLoader.load(fieldJson());
        check("version", field.version == 1);
        check("calibrated", field.calibrated);
        check("shelf heading", field.shelfApproach.heading == 90.0);
        check("four slots", field.factorySlots.length == 4);
        check("fac1 y", field.shelfFacs[0].y == 32.99);
        check("fac2 y", field.shelfFacs[1].y == 100.42);
        check("fac3 y", field.shelfFacs[2].y == 167.65);

        LiftingSequenceConfig blue = merged(Alliance.BLUE);
        check("blue 01 top dcm", blue.factoryFor("01").placement.y == 167.95);
        check("blue 02 vang", blue.factoryFor("02").placement.y == 133.63);
        check("blue 03 xanhla", blue.factoryFor("03").placement.y == 66.65);
        check("blue 04 bottom do", blue.factoryFor("04").placement.y == 32.58);
        check("blue factory heading", blue.factoryFor("01").placement.heading == -90.0);

        LiftingSequenceConfig red = merged(Alliance.RED);
        check("red 01 bottom do", red.factoryFor("01").placement.y == 32.58);
        check("red 04 top dcm", red.factoryFor("04").placement.y == 167.95);
        check("red keeps code", red.factoryFor("02").type.equals("02"));

        try {
            FieldBlueConfigLoader.load(fieldJson().replace("\"calibrated\": true", ""));
            check("missing calibrated rejected", false);
        } catch (IllegalArgumentException expected) {
            check("missing calibrated rejected", true);
        }

        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
