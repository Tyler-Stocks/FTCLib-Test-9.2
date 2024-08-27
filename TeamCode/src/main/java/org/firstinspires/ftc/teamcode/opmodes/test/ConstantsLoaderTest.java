package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.constants.Constants.OdometryConstants.OFFSET;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;

@TeleOp(name = "Test - Constants Loader", group = "Test")
public final class ConstantsLoaderTest extends OpModeTemplate {

    @Override public void initialize() {
       new ConstantsLoader(telemetry).load();

       telemetry.addData("OFFSET X", OFFSET.x);
       telemetry.addData("OFFSET Y", OFFSET.y);
       telemetry.addData("OFFSET H", OFFSET.h);

       telemetry.update();
    }
}
