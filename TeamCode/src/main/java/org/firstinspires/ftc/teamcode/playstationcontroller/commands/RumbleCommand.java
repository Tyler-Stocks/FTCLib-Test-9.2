package org.firstinspires.ftc.teamcode.playstationcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleCommand extends CommandBase {
	private final Gamepad[] gamepads;

	private final int durationMS;
	private final double intensity;

	public RumbleCommand(@NonNull Gamepad ... gamepads) {
		this.gamepads = gamepads;
		this.durationMS = 1000;
		this.intensity  = 1;
	}

	public RumbleCommand(int durationMS, @NonNull Gamepad ... gamepads) {
		this.gamepads = gamepads;
		this.durationMS = durationMS;
		this.intensity  = 1;
	}

	public RumbleCommand(int durationMS, double intensity, @NonNull Gamepad ... gamepads) {
		this.gamepads   = gamepads ;
		this.durationMS = durationMS;
		this.intensity  = intensity;
	}

	@Override public void execute() {
		for (Gamepad gamepad : gamepads) {
			gamepad.rumble(intensity, intensity, durationMS);
		}
	}

	@Override public boolean isFinished() {
		return true;
	}
}
