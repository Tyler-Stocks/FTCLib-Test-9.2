package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide;

public class OpenOuttakeCommand extends CommandBase {
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
	public OpenOuttakeCommand(
			@NonNull ArmSubsystem armSubsystem,
			@NonNull RobotSide outtakeSide,
			double position) {
		this.armSubsystem = armSubsystem;
		this.outtakeSide  = outtakeSide;
		this.position     = position;
	}

	@Override public void execute() {
		armSubsystem.setOuttakePosition(outtakeSide, position);
	}

	@Override public boolean isFinished()	 {
		return true;
	}
}
