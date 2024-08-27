package org.firstinspires.ftc.teamcode.subsystems.hanger.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.hanger.HangerSubsystem;

public class ReleaseHangerCommand extends CommandBase {
	private final HangerSubsystem hangerSubsystem;

	private boolean isFinished;

	public ReleaseHangerCommand(@NonNull HangerSubsystem hangerSubsystem) {
		this.hangerSubsystem = hangerSubsystem;

		isFinished = false;
	}

	@Override public void execute() {
		hangerSubsystem.release();
		isFinished = true;
	}

	@Override public boolean isFinished() {
		return isFinished;
	}
}
