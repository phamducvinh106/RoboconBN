package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** B = blue, X = red; hold through start. */
public final class AllianceSelector {
    private AllianceSelector() {}

    public static Alliance select(LinearOpMode opMode) {
        Alliance selected = Alliance.BLUE;
        while (!opMode.isStopRequested() && !opMode.opModeIsActive()) {
            if (opMode.gamepad1.b) selected = Alliance.BLUE;
            if (opMode.gamepad1.x) selected = Alliance.RED;
            opMode.telemetry.addLine("Alliance: B=BLUE  X=RED");
            opMode.telemetry.addData("selected", selected.name());
            opMode.telemetry.update();
            opMode.sleep(50);
        }
        return selected;
    }
}
