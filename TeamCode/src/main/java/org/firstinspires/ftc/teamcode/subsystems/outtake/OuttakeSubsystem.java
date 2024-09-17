package org.firstinspires.ftc.teamcode.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class OuttakeSubsystem extends SubsystemBase {
	private final ServoImplEx leftDoorServo, rightDoorServo;

	private Telemetry telemetry;

	public double leftOuttakeDoorPosition, rightOuttakeDoorPosition;

	public OuttakeSubsystem(@NonNull OpMode opMode) {
		telemetry = opMode.telemetry;

		this.leftDoorServo  = opMode.hardwareMap.get(ServoImplEx.class, LEFT_OUTTAKE_SERVO_NAME);
		this.rightDoorServo = opMode.hardwareMap.get(ServoImplEx.class, RIGHT_OUTTAKE_SERVO_NAME);

		leftOuttakeDoorPosition  = 0.0;
		rightOuttakeDoorPosition = 0.0;
	}

	@Override public void periodic() {
		leftDoorServo.setPosition(leftOuttakeDoorPosition);
		rightDoorServo.setPosition(rightOuttakeDoorPosition);
	}

	/**
	 * Sets the left outtake door to the open position
	 */
	public void openLeftOuttakeDoor() {
		leftOuttakeDoorPosition = OPEN_POSITION;
	}

	/**
	 * Sets the right outtake door to the open position
	 */
	public void openRightOuttakeDoor() {
		rightOuttakeDoorPosition = OPEN_POSITION;
	}

	/**
	 * Sets the left outtake door to the closed position
	 */
	public void closeLeftOuttakeDoor() {
		leftOuttakeDoorPosition = CLOSED_POSITION;
	}

	/**
	 * Sets the right outtake door to the closed position
	 */
	public void closeRightOuttakeDoor() {
		rightOuttakeDoorPosition = CLOSED_POSITION;
	}

	public void setLeftOuttakeDoorPosition(double position) {
		leftOuttakeDoorPosition = position;
	}

	public void setRightOuttakeDoorPosition(double position) {
		rightOuttakeDoorPosition = position;
	}

	/**
	 * Sets both outtake doors to the position specified by the "position" argument
	 * @param position The position to set the outtake doors to.
	 */
	public void setOuttakeDoorPosition(double position) {
		leftOuttakeDoorPosition = position;
		rightOuttakeDoorPosition = position;
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
