package org.firstinspires.ftc.teamcode.test;

import java.nio.file.Files;
import java.nio.file.Paths;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfigLoader;

public final class LiftingSequenceConfigTest {
    static int passed;
    static void check(String name, boolean ok) { if (!ok) throw new AssertionError(name); passed++; }
    static String valid() throws Exception { return new String(Files.readAllBytes(Paths.get("TeamCode/src/main/assets/phase2-lifting-config.json")), "UTF-8"); }
    static void rejects(String name, String json) { try { LiftingSequenceConfigLoader.load(json); throw new AssertionError(name); } catch (IllegalArgumentException expected) { passed++; } }
    public static void main(String[] args) throws Exception {
        String json=valid(); LiftingSequenceConfig c=LiftingSequenceConfigLoader.load(json);
        check("version", c.version == 1);
        check("calibration", c.lift2Steps == 5625);
        check("factories", c.factoryFor("01").placement.x == 120);
        check("fingerprint", c.fingerprint.length() == 8);
        check("identities", c.webcam1Identity.equals("webcam1") && c.webcam2Identity.equals("webcam2"));
        check("pi5 i2c", c.pi5I2cAddress == 66);
        check("camera width", c.cameraFrameWidth == 320);
        rejects("missing", json.replace("\"lift2Steps\": 5625,", ""));
        rejects("wrong version", json.replace("\"version\": 1", "\"version\": 2"));
        rejects("range", json.replace("\"centerSpeed\": 0.15", "\"centerSpeed\": 2"));
        rejects("malformed", json.substring(0, json.lastIndexOf('}')));
        System.out.println("RESULT: "+passed+" passed, 0 failed");
    }
}
