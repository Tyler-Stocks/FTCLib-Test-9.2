package org.firstinspires.ftc.teamcode.subsystems.intake.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class BackBeamBreakTrigger extends Trigger {
    private final IntakeSubsystem intakeSubsystem;

    public BackBeamBreakTrigger(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override public boolean get() {
        return intakeSubsystem.backBeamBreakIsPressed();
    }
}
