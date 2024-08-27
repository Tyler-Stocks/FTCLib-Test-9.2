package org.firstinspires.ftc.teamcode.subsystems.intake.triggers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class IntakeIsActiveTrigger extends Trigger {
	private final IntakeSubsystem intakeSubsystem;

	public IntakeIsActiveTrigger(@NonNull IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
	}

	@Override public boolean get() {
		return intakeSubsystem.isActive();
	}
}
