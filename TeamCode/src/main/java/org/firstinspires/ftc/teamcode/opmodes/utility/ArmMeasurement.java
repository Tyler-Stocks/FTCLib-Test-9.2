package org.firstinspires.ftc.teamcode.opmodes.utility;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.playstationcontroller.PlayStationController.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.utility.MotorUtility;

@TeleOp(name = "Utility - Arm Measurement", group = "Utility")
public final class ArmMeasurement extends CommandOpMode {

    private DcMotorImplEx elevatorMotor, wormMotor;

    @Override public void initialize() {
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, ELEVATOR_MOTOR_NAME);
        wormMotor     = hardwareMap.get(DcMotorImplEx.class, WORM_MOTOR_NAME);

        elevatorMotor.setDirection(REVERSE);

        resetMotors();

        configureBindings();

        schedule(
                new RunCommand(this::runArmMotorsManual),
                new RunCommand(this::displayArmMotorPositions),
                new RunCommand(telemetry::update)
        );
    }

    private void configureBindings() {
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        new GamepadButton(operatorGamepad, OPTIONS)
                .whenPressed(new RunCommand(this::resetMotors));
    }

    private void resetMotors() {
        MotorUtility.reset(elevatorMotor, wormMotor);
    }

    private void displayArmMotorPositions() {
        telemetry.addData("Elevator Position", elevatorMotor.getCurrentPosition());
        telemetry.addData("Elevator Power", elevatorMotor.getPower());
        telemetry.addData("Elevator Velocity", elevatorMotor.getVelocity(DEGREES));
        telemetry.addData("Worm Position", wormMotor.getCurrentPosition());
        telemetry.addData("Worm Power", wormMotor.getPower());
        telemetry.addData("Worm Velocity", wormMotor.getVelocity(DEGREES));
    }

    private void runArmMotorsManual() {
        elevatorMotor.setPower(gamepad1.left_stick_x);
        wormMotor.setPower(gamepad1.left_stick_y);
    }
}
