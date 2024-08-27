package org.firstinspires.ftc.teamcode.subsystems.purplepixelplacer;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.constants.Constants.PurplePixelPlacerConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * <h1>Purple Pixel Placer Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate the servo which controls the purple pixel placement. Contains the
 *     following hardware
 *     <ul>
 *         <li>Purple Pixel Placer Servo</li>
 *     </ul>
 * </p>
 */
public final class PurplePixelPlacerSubsystem extends SubsystemBase {
    private final ServoImplEx purplePixelPlacerServo;

    private final Telemetry telemetry;

    private double purplePixelPlacerServoPosition;

    public PurplePixelPlacerSubsystem(@NonNull OpMode opMode) {
       telemetry = opMode.telemetry;

       HardwareMap hardwareMap = opMode.hardwareMap;

       purplePixelPlacerServo = hardwareMap.get(ServoImplEx.class, PURPLE_PIXEL_PLACER_SERVO_NAME);

       purplePixelPlacerServo.setDirection(PURPLE_PIXEL_PLACER_SERVO_DIRECTION);

       moveToNeutralPosition(); // Bypass the normal update loop for initialization

       purplePixelPlacerServoPosition = PURPLE_PIXEL_PLACER_NEUTRAL_POSITION;
    }

    @Override public void periodic() {
       purplePixelPlacerServo.setPosition(purplePixelPlacerServoPosition);
    }

    /**
     * Moves the purple pixel placer to the neutral position
     */
    public void moveToNeutralPosition() {
        purplePixelPlacerServoPosition = PURPLE_PIXEL_PLACER_NEUTRAL_POSITION;
    }

    /**
     * Moves the purple pixel placer to the active position
     */
    public void moveToActivePosition() {
        purplePixelPlacerServoPosition = PURPLE_PIXEL_PLACER_ACTIVE_POSITION;
    }

    /**
     * Moves the purple pixel placer to the input position
     * @param position The position to move the purple pixel placer servo to
     */
    public void moveToPosition(double position) {
        purplePixelPlacerServoPosition = position;
    }
}
