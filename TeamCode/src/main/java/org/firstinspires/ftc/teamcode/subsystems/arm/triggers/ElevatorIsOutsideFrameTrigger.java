package org.firstinspires.ftc.teamcode.subsystems.arm.triggers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class ElevatorIsOutsideFrameTrigger extends Trigger {
	private final ArmSubsystem armSubsystem;

	/**
	 * Returns true when the elevator is outside of the frame, and false when it is inside
	 * @param armSubsystem The arm subsystem to read from
	 */
	public ElevatorIsOutsideFrameTrigger(@NonNull ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
	}

	@Override public boolean get() {
		return !armSubsystem.elevatorInFrame();
	}
}
