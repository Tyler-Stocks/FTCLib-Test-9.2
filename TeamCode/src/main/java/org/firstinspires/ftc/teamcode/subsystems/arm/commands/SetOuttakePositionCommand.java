package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide;

public class SetOuttakePositionCommand extends CommandBase {
	private final ArmSubsystem armSubsystem;
	private final RobotSide outtakeSide;
	private final double position;

	/**
	 * Sets the position of the outtake. Note that if the arm is inside the frame, but not at home
	 * the outtake cannot open.
	 * @param armSubsystem The arm subsystem to open the door of
	 * @param outtakeSide Which outtake door to open (Left, Right, or Both)
	 * @param position The position to move the specified outtake door(s) to
	 */
	public SetOuttakePositionCommand(
			@NonNull ArmSubsystem armSubsystem,
			@NonNull RobotSide outtakeSide,
			double position) {
		this.armSubsystem = armSubsystem;
		this.outtakeSide  = outtakeSide;
		this.position     = position;
	}

	@Override public void execute() {
		switch (outtakeSide) {
			case LEFT:
				armSubsystem.outtake.setLeftOuttakeDoorPosition(position);
				break;
			case RIGHT:
				armSubsystem.outtake.setRightOuttakeDoorPosition(position);
				break;
			case BOTH:
				armSubsystem.outtake.setOuttakePosition(position);
				break;
		}
	}

	@Override public boolean isFinished() {
		return true;
	}
}
