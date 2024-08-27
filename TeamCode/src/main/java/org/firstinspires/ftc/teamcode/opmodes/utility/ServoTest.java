package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.constants.Constants.ServoTestConstants.*;
import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.*;

import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;

@TeleOp(name = "Utility - Servo Position Tester", group = "Utility")
public class ServoTest extends CommandOpMode{
	private ServoImplEx servo;

	private double servoPosition;

	@Override public void initialize() {
		new ConstantsLoader().load();

		servoPosition = TEST_SERVO_START_POSITION;

		servo = hardwareMap.get(ServoImplEx.class, TEST_SERVO_NAME);

		configureBindings();

		servo.setPosition(TEST_SERVO_START_POSITION);
		servo.setDirection(TEST_SERVO_DIRECTION);

		schedule(
			new RunCommand(this::setServoPosition),
			new RunCommand(this::displayServoPosition),
			new RunCommand(telemetry::update)
		);
	}

	public void configureBindings() {
		GamepadEx operatorGamepad = new GamepadEx(gamepad2);

		new GamepadButton(operatorGamepad, OPTIONS)
				.whenPressed(this::resetServoPosition);

		new GamepadButton(operatorGamepad, DPAD_UP)
				.whenPressed(this::incrementServoPosition);

		new GamepadButton(operatorGamepad, DPAD_DOWN)
				.whenPressed(this::decrementServoPosition);
	}

	private void resetServoPosition() {
		servoPosition = 0.0;
	}

	private void incrementServoPosition() {
		servoPosition += 0.01;
	}

	private void decrementServoPosition() {
		servoPosition -= 0.01;
	}

	private void setServoPosition() {
		servo.setPosition(servoPosition);
	}

	private void displayServoPosition() {
		telemetry.addData("Servo Position (Local)", servoPosition);
		telemetry.addData("Servo Position", servo.getPosition());
	}
}
