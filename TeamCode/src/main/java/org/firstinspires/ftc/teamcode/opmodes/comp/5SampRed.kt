package org.firstinspires.ftc.teamcode.opmodes.comp

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.util.SampleColor

@Autonomous
class `5SampRed`: `5SampBlue`() {

    init {
        VALID_COLORS = listOf(SampleColor.RED, SampleColor.YELLOW)
    }

}