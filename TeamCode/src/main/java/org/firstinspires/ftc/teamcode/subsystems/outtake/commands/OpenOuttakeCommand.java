package org.firstinspires.ftc.teamcode.subsystems.outtake.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide;

public class OpenOuttakeCommand extends CommandBase {
	private final OuttakeSubsystem outtakeSubsystem;
	private final RobotSide outtakeSide;
	private final double position;

	/**
	 * Sets the position of the outtake. Note that if the arm is inside the frame, but not at home
	 * the outtake cannot open.
	 * @param outtakeSubsystem The arm subsystem to open the door of
	 * @param outtakeSide Which outtake door to open (Left, Right, or Both)
	 * @param position The position to move the specified outtake door(s) to
	 */
	public OpenOuttakeCommand(
			@NonNull OuttakeSubsystem outtakeSubsystem,
			@NonNull RobotSide outtakeSide,
			double position) {
		this.outtakeSubsystem = outtakeSubsystem;
		this.outtakeSide  = outtakeSide;
		this.position     = position;

		addRequirements(outtakeSubsystem);
	}

	@Override public void execute() {
		switch (outtakeSide) {
			case LEFT:
				outtakeSubsystem.setLeftOuttakeDoorPosition(position);
				break;
			case RIGHT:
				outtakeSubsystem.setRightOuttakeDoorPosition(position);
				break;
			case BOTH:
				outtakeSubsystem.setOuttakeDoorPosition(position);
				break;
		}
	}

	@Override public boolean isFinished()	 {
		return true;
	}
}
