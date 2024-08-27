package org.firstinspires.ftc.teamcode.subsystems.mosaicfixers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.constants.Constants.MosaicFixerConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerPosition.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * <h1>Mosaic Fixer Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate the mosaic fixer servos. Contains the following hardware
 *     <ul>
 *         <li>Left Mosaic Fixer Servo</li>
 *         <li>Right Mosaic Fixer Servo</li>
 *     </ul>
 * </p>
 */
public class MosaicFixerSubsystem extends SubsystemBase {
    private final ServoImplEx leftMosaicFixerServo,
                              rightMosaicFixerServo;

    private final Telemetry telemetry;

    private MosaicFixerPosition leftMosaicFixerPosition, rightMosaicFixerPosition;

    private boolean enableLeftMosaicFixer, enableRightMosaicFixer;

    /**
     * Constructs a new MosaicFixerSubsystem
     * @param opMode The opMode you are running ; To obtain the hardwareMap and telemetry objects
     */
    public MosaicFixerSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        leftMosaicFixerServo
                = opMode.hardwareMap.get(ServoImplEx.class, LEFT_MOSAIC_FIXER_SERVO_NAME);
        rightMosaicFixerServo
                = opMode.hardwareMap.get(ServoImplEx.class, RIGHT_MOSAIC_FIXER_SERVO_NAME);

        leftMosaicFixerServo.setDirection(LEFT_MOSAIC_FIXER_SERVO_DIRECTION);
        rightMosaicFixerServo.setDirection(RIGHT_MOSAIC_FIXER_SERVO_DIRECTION);

        enableLeftMosaicFixer  = true;
        enableRightMosaicFixer = true;

        leftMosaicFixerPosition  = RETRACTED;
        rightMosaicFixerPosition = RETRACTED;

        leftMosaicFixerServo.setPosition(LEFT_MOSAIC_FIXER_RETRACTED_POSITION);
        rightMosaicFixerServo.setPosition(RIGHT_MOSAIC_FIXER_RETRACTED_POSITION);
    }

    @Override public void periodic() {
        if (enableLeftMosaicFixer) {
            leftMosaicFixerServo.setPwmEnable();
        } else {
            leftMosaicFixerServo.setPwmDisable();
            return;
        }

        if (enableRightMosaicFixer) {
            rightMosaicFixerServo.setPwmEnable();
        } else {
            rightMosaicFixerServo.setPwmDisable();
            return;
        }

        switch (leftMosaicFixerPosition) {
            case RETRACTED:
                leftMosaicFixerServo.setPosition(LEFT_MOSAIC_FIXER_RETRACTED_POSITION);
                break;
            case LOW:
                leftMosaicFixerServo.setPosition(LEFT_MOSAIC_FIXER_LOW_POSITION);
                break;
            case MEDIUM:
                leftMosaicFixerServo.setPosition(LEFT_MOSAIC_FIXER_MEDIUM_POSITION);
                break;
            case HIGH:
                leftMosaicFixerServo.setPosition(LEFT_MOSAIC_FIXER_HIGH_POSITION);
                break;
        }

        switch (rightMosaicFixerPosition) {
            case RETRACTED:
                rightMosaicFixerServo.setPosition(RIGHT_MOSAIC_FIXER_RETRACTED_POSITION);
                break;
            case LOW:
                rightMosaicFixerServo.setPosition(RIGHT_MOSAIC_FIXER_MEDIUM_POSITION);
                break;
            case HIGH:
                rightMosaicFixerServo.setPosition(RIGHT_MOSAIC_FIXER_HIGH_POSITION);
                break;
            default: // Right mosaic fixer has no medium position
                break;
        }
    }


    /**
     * Signals to the subsystem to disable the left mosaic fixer
     */
    public void disableLeftMosaicFixer() {
        enableLeftMosaicFixer = false;
    }

    /**
     * Signals to the subsystem to disable the right mosaic fixer
     */
    public void disableRightMosaicFixer() {
        enableRightMosaicFixer = false;
    }

    /**
     * Signals to the subsystem to enable the left mosaic fixer
     */
    public void enableLeftMosaicFixer() {
        enableLeftMosaicFixer = true;
    }

    /**
     * Signals to the subsystem to enable the right mosaic fixer
     */
    public void enableRightMosaicFixer() {
        enableRightMosaicFixer = true;
    }

    /**
     * Signals to the subsystem that it should move the left mosaic fixer to the input position
     * @param mosaicFixerPosition The position to move to
     */
    public void moveLeftMosaicFixerToPosition(@NonNull MosaicFixerPosition mosaicFixerPosition) {
       leftMosaicFixerPosition = mosaicFixerPosition;
    }

    /**
     * Signals to the subsystem that it should move the right mosaic fixer to the input position
     * @param mosaicFixerPosition The position to move to
     */
    public void moveRightMosaicFixerToPosition(@NonNull MosaicFixerPosition mosaicFixerPosition) {
       rightMosaicFixerPosition = mosaicFixerPosition;
    }

    /**
     * Displays debug information about the mosaic fixers
     */
    public void debug() {
        telemetry.addLine("----- Mosaic Fixer Debug -----");
        telemetry.addData("Left Mosaic Fixer Position", leftMosaicFixerServo.getPosition());
        telemetry.addData("Right Mosaic Fixer Position", rightMosaicFixerServo.getPosition());
        telemetry.addData("Left Mosaic Fixer Direction", leftMosaicFixerServo.getDirection());
        telemetry.addData("Right Mosaic Fixer Direction", rightMosaicFixerServo.getDirection());
        telemetry.addData("Left Mosaic Fixer PWM Enabled", leftMosaicFixerServo.isPwmEnabled());
        telemetry.addData("Right Mosaic Fixer PWM Enabled", rightMosaicFixerServo.isPwmEnabled());
        telemetry.addData("Left Mosaic Fixer Enabled (Local)", enableLeftMosaicFixer);
        telemetry.addData("Right Mosaic Fixer Enabled (Local)", enableRightMosaicFixer);
        telemetry.addData("Left Mosaic Fixer State", leftMosaicFixerPosition);
        telemetry.addData("Right Mosaic Fixer State", rightMosaicFixerPosition);
    }
}
