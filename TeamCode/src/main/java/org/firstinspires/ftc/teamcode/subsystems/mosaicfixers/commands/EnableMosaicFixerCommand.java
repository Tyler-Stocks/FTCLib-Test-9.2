package org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide;

public final class EnableMosaicFixerCommand extends CommandBase {
	private final MosaicFixerSubsystem mosaicFixerSubsystem;
	private final RobotSide            mosaicFixerSide;

	private boolean isFinished;

	public EnableMosaicFixerCommand(
			@NonNull MosaicFixerSubsystem mosaicFixerSubsystem,
			@NonNull RobotSide mosaicFixerSide
	) {
		this.mosaicFixerSubsystem = mosaicFixerSubsystem;
		this.mosaicFixerSide      = mosaicFixerSide;

		isFinished = false;

		addRequirements(mosaicFixerSubsystem);
	}

	@Override public void execute() {
		switch (mosaicFixerSide) {
			case LEFT:
				mosaicFixerSubsystem.enableLeftMosaicFixer();
				break;
			case RIGHT:
				mosaicFixerSubsystem.enableRightMosaicFixer();
				break;
			case BOTH:
				mosaicFixerSubsystem.enableLeftMosaicFixer();
				mosaicFixerSubsystem.enableRightMosaicFixer();
				break;

		}

		isFinished = true;
	}

	@Override public boolean isFinished() {
		return isFinished;
	}
}
