package org.firstinspires.ftc.teamcode.subsystems.hanger;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.constants.Constants.HangerConstants.*;

/**
 * <h1>Hanger Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to control the release of the hook during endgame. Encapsulates the following
 *     hardware:
 *     <ul>
 *         <li>Hanger Servo</li>
 *     </ul>
 * </p>
 */
public class HangerSubsystem extends SubsystemBase {
    private final Servo hangerServo;

    private final Telemetry telemetry;

    /**
     * Constructs a new HangerSubsystem
     * @param opMode The opmode you are running ; To obtain the hardware map and telemetry objects
     */
    public HangerSubsystem(@NonNull OpMode opMode) {
        hangerServo = opMode.hardwareMap.get(Servo.class, HANGER_SERVO_NAME);

        telemetry = opMode.telemetry;
    }

    /**
     * Sets the hanger servo to the release position
     */
    public void release() {
        hangerServo.setPosition(HANG_POSITION);
    }

    /**
     * Resets the hanger to the zero position
     */
    public void reset() {
        hangerServo.setPosition(ZERO_POSITION);
    }

    /**
     * Displays debug information about the hanger
     */
    public void debug() {
        telemetry.addLine("---- Hanger Debug ----");
        telemetry.addData("Position", hangerServo.getPosition());
        telemetry.addData("Direction", hangerServo.getDirection());
    }
}
