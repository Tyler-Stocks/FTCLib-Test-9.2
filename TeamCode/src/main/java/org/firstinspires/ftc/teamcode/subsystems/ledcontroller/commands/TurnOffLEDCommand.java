package org.firstinspires.ftc.teamcode.subsystems.ledcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState;
import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDSubsystem;

public class TurnOffLEDCommand extends CommandBase {
	private final LEDSubsystem ledSubsystem;

	private boolean isFinished;

	public TurnOffLEDCommand(@NonNull LEDSubsystem ledSubsystem) {
		this.ledSubsystem = ledSubsystem;

		isFinished = false;

		addRequirements(ledSubsystem);
	}

	@Override public void execute() {
		ledSubsystem.setBothLEDStates(LEDState.OFF);

		isFinished = true;
	}

	@Override public boolean isFinished() {
		return isFinished;
	}
}
