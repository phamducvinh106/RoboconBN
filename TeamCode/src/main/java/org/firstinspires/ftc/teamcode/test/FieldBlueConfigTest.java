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
        check("calibrated flag present", !field.calibrated);
        check("shelf heading", field.shelfApproach.heading == 90.0);
        check("four slots", field.factorySlots.length == 4);

        LiftingSequenceConfig blue = merged(Alliance.BLUE);
        check("blue 01 top", blue.factoryFor("01").near.y == 40.0);
        check("blue 02 second", blue.factoryFor("02").near.y == 50.0);
        check("blue 03 third", blue.factoryFor("03").near.y == 60.0);
        check("blue 04 bottom", blue.factoryFor("04").near.y == 70.0);

        LiftingSequenceConfig red = merged(Alliance.RED);
        check("red 01 bottom", red.factoryFor("01").near.y == 70.0);
        check("red 02 third", red.factoryFor("02").near.y == 60.0);
        check("red 03 second", red.factoryFor("03").near.y == 50.0);
        check("red 04 top", red.factoryFor("04").near.y == 40.0);
        check("red keeps code", red.factoryFor("02").type.equals("02"));

        try {
            FieldBlueConfigLoader.load(fieldJson().replace("\"calibrated\": false", ""));
            check("missing calibrated rejected", false);
        } catch (IllegalArgumentException expected) {
            check("missing calibrated rejected", true);
        }

        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
