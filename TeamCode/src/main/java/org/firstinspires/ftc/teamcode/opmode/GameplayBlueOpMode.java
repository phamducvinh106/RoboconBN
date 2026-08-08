package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.core.Alliance;

@Autonomous(name = "Gameplay Blue", group = "Autonomous")
public final class GameplayBlueOpMode extends LiftingSequenceOpMode {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
