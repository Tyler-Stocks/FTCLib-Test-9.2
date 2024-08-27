package org.firstinspires.ftc.teamcode.subsystems.launcher.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherSubsystem;

public class ReleaseLauncherCommand extends CommandBase {
	private final LauncherSubsystem launcherSubsystem;

	private boolean isFinished;

	public ReleaseLauncherCommand(@NonNull LauncherSubsystem launcherSubsystem) {
		this.launcherSubsystem = launcherSubsystem;

		isFinished = false;

		addRequirements(launcherSubsystem);
	}

	@Override public void execute() {
		launcherSubsystem.launch();
		isFinished = true;
	}

	@Override public boolean isFinished() {
		return isFinished;
	}
}
