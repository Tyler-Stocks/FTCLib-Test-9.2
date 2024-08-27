package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.*;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.IntakeConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * <h1>Intake Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate intake. Encapsulates the following
 *     hardware:
 *     <ul>
 *         <li>Intake Motor</li>
 *         <li>Front Beam Break</li>
 *         <li>Back Beam Break</li>
 *     </ul>
 * </p>
 */
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorImplEx intakeMotor;

    private final DigitalChannel frontBeamBreak,
                                 backBeamBreak;

    private final Telemetry telemetry;

    private boolean shouldIntake, shouldOuttake;

    private boolean isActive;

    private boolean disableIntaking;

    private boolean isIntaking, isOuttaking;

    /**
     * Constructs a new intake subsystem
     * @param opMode The opMode you are running ; To obtain the hardwareMap and telemetry objects
     */
    public IntakeSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, INTAKE_MOTOR_NAME);

        frontBeamBreak = hardwareMap.get(DigitalChannel.class, FRONT_BEAM_BREAK_NAME);
        backBeamBreak  = hardwareMap.get(DigitalChannel.class, BACK_BEAM_BREAK_NAME);

        frontBeamBreak.setMode(INPUT);
        backBeamBreak.setMode(INPUT);

        intakeMotor.setZeroPowerBehavior(FLOAT);

        isActive        = false;
        shouldIntake    = false;
        shouldOuttake   = false;
        disableIntaking = false;
    }

    @Override public void periodic() {
        isActive = intakeMotor.isBusy();

        // Stop the intake motor if we try to intake and outtake at the same time
        if (shouldIntake && shouldOuttake) {
            intakeMotor.setPower(0.0);
            return;
        }

        if (shouldIntake && !disableIntaking) {
            intakeMotor.setDirection(REVERSE);
            intakeMotor.setPower(INTAKE_POWER);
        }

        if (shouldOuttake) {
            intakeMotor.setDirection(FORWARD);
            intakeMotor.setPower(OUTTAKE_POWER);
        }

        if (!shouldIntake && !shouldOuttake) intakeMotor.setPower(0.0);

        isActive = intakeMotor.getPower() != 0;

        isIntaking  = isActive && intakeMotor.getDirection() == FORWARD;
        isOuttaking = isActive && intakeMotor.getDirection() == REVERSE;
    }

    /**
     * Signals to the subsystem that it should be intaking.
     * Note this function can be overwritten if both intake
     * and outtake are called at the same time.
     */
    public void intake() {
        shouldIntake = true;
    }

    /**
     * Signals to the subsystem that is should be outtaking.
     * Note this function can be overwritten if both intake
     * and outtake are called at the same time.
     */
    public void outtake() {
        shouldOuttake = true;
    }

    /**
     * Signals to the subsystem that it should be stopping.
     * Note this cannot be overwritten.
     */
    public void stop() {
        shouldIntake  = false;
        shouldOuttake = false;
    }

    public void disableIntaking() {
       disableIntaking = true;
    }

    public void enableIntaking() {
       disableIntaking = false;
    }

    /**
     * @return Whether the front beam break is pressed
     */
    public boolean frontBeamBreakIsPressed() {
        return !frontBeamBreak.getState();
    }

    /**
     * @return Whether the back beam break is pressed
     */
    public boolean backBeamBreakIsPressed() {
        return !backBeamBreak.getState();
    }

    /**
     * @return Whether the intake is intaking based on the direction and power of the intake motor.
     */
    public boolean isIntaking() {
        return isIntaking;
    }

    /**
     * @return Whether the intake is outtaking based on the direction and power of the intake motor.
     */
    public boolean isOuttaking() {
        return isOuttaking;
    }

    /**
     * @return Whether the intake is active based on the direction and power of the motor.
     */
    public boolean isActive() {
        return isActive;
    }

    /**
     * Displays debug information about the intake.
     */
    public void debug() {
        telemetry.addLine("---- Intake Telemetry ----");
        telemetry.addData("Direction", intakeMotor.getDirection());
        telemetry.addData("Power", intakeMotor.getPower());
        telemetry.addData("Current (AMPS)", intakeMotor.getCurrent(AMPS));
    }
}
