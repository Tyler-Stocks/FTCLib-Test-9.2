package org.firstinspires.ftc.teamcode.subsystems.arm;

import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Outtake {
	private final ServoImplEx leftDoorServo, rightDoorServo;

	private Telemetry telemetry;

	public double leftOuttakeDoorPosition, rightOuttakeDoorPosition;

	public Outtake(@NonNull OpMode opMode) {
		telemetry = opMode.telemetry;

		this.leftDoorServo  = opMode.hardwareMap.get(ServoImplEx.class, LEFT_OUTTAKE_SERVO_NAME);
		this.rightDoorServo = opMode.hardwareMap.get(ServoImplEx.class, RIGHT_OUTTAKE_SERVO_NAME);

		leftOuttakeDoorPosition  = 0.0;
		rightOuttakeDoorPosition = 0.0;
	}

	/**
	 * Sets the left outtake door to the open position
	 */
	public void openLeftOuttakeDoor() {
		setLeftOuttakeDoorPosition(OPEN_POSITION);
	}

	/**
	 * Sets the right outtake door to the open position
	 */
	public void openRightOuttakeDoor() {
		setRightOuttakeDoorPosition(OPEN_POSITION);
	}

	/**
	 * Sets the left outtake door to the closed position
	 */
	public void closeLeftOuttakeDoor() {
		setLeftOuttakeDoorPosition(CLOSED_POSITION);
	}

	/**
	 * Sets the right outtake door to the closed position
	 */
	public void closeRightOuttakeDoor() {
		setRightOuttakeDoorPosition(CLOSED_POSITION);
	}

	/**
	 * Sets the left outtake door to the position specified by the "position" argument
	 * @param position The position to set the left outtake door to
	 */
	public void setLeftOuttakeDoorPosition(double position) {
		leftDoorServo.setPosition(position);
	}

	/**
	 * Sets the right outtake door to the position specified by the "position" argument
	 * @param position The position to set the left outtake door to
	 */
	public void setRightOuttakeDoorPosition(double position) {
		rightDoorServo.setPosition(position);
	}

	/**
	 * Sets both outtake doors to the position specified by the "position" argument
	 * @param position The position to set the outtake doors to.
	 */
	public void setOuttakePosition(double position) {
		setLeftOuttakeDoorPosition(position);
		setRightOuttakeDoorPosition(position);
	}

	/**
	 * Opens both outtake doors.
	 */
	public void openOuttake() {
		openLeftOuttakeDoor();
		openRightOuttakeDoor();
	}

	/**
	 * Closes both outtake doors
	 */
	public void closeOuttake() {
		closeLeftOuttakeDoor();
		closeRightOuttakeDoor();
	}
}
