package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsSaver;

@TeleOp(name = "Utility - Constants Saver", group = "Utility")
public final class ConstantsSaverUtility extends CommandOpMode {
    private ConstantsSaver constantsSaver;

    @Override public void initialize() {
        constantsSaver = new ConstantsSaver(telemetry);

        configureTriggers();
    }

    private void configureTriggers() {
        new Trigger(this::isStarted)
                .whenActive(this::saveConstants);
    }

    private void saveConstants() {
        constantsSaver.save();
        telemetry.addLine("Saved Constants");
    }
}
