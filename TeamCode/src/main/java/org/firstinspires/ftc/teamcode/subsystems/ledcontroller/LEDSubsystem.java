package org.firstinspires.ftc.teamcode.subsystems.ledcontroller;

import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.OUTPUT;
import static org.firstinspires.ftc.teamcode.constants.Constants.LEDControllerConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState.OFF;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * <h1>LED Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate the four digital channels that control the LED's on the robot.
 *     Contains the following hardware
 *     <ul>
 *         <li>Red Channel Left</li>
 *         <li>Green Channel Left</li>
 *         <li>Red Channel Right</li>
 *         <li>Green Channel Right</li>
 *     </ul>
 * </p>
 */
public class LEDSubsystem extends SubsystemBase {
    private final DigitalChannelImpl leftLEDRedChannel,
                                     leftLEDGreenChannel,
                                     rightLEDRedChannel,
                                     rightLEDGreenChannel;

    private final Telemetry telemetry;

    private LEDState leftLEDState,
                     rightLEDState;

    /**
     * Constructs a new LEDSubsystem
     * @param opMode The opMode you are running ; To obtain the hardwareMap and telemetry
     */
    public LEDSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        leftLEDRedChannel
                = hardwareMap.get(DigitalChannelImpl.class, LEFT_LED_RED_CHANNEL_NAME);
        leftLEDGreenChannel
                = hardwareMap.get(DigitalChannelImpl.class, LEFT_LED_GREEN_CHANNEL_NAME);
        rightLEDRedChannel
                = hardwareMap.get(DigitalChannelImpl.class, RIGHT_LED_RED_CHANNEL_NAME);
        rightLEDGreenChannel
                = hardwareMap.get(DigitalChannelImpl.class, RIGHT_LED_GREEN_CHANNEL_NAME);

        leftLEDRedChannel.setMode(OUTPUT);
        leftLEDGreenChannel.setMode(OUTPUT);
        rightLEDRedChannel.setMode(OUTPUT);
        rightLEDGreenChannel.setMode(OUTPUT);

        leftLEDRedChannel.setState(false);
        leftLEDGreenChannel.setState(false);
        rightLEDRedChannel.setState(false);
        rightLEDGreenChannel.setState(false);

        leftLEDState  = OFF;
        rightLEDState = OFF;
    }

    @Override public void periodic() {
        switch (leftLEDState) {
            case RED:
                leftLEDRedChannel.setState(true);
                leftLEDGreenChannel.setState(false);
                break;
            case GREEN:
                leftLEDRedChannel.setState(false);
                leftLEDGreenChannel.setState(true);
                break;
            case AMBER:
                leftLEDRedChannel.setState(true);
                leftLEDGreenChannel.setState(true);
                break;
            case OFF:
                leftLEDRedChannel.setState(false);
                leftLEDGreenChannel.setState(false);
                break;
        }

        switch (rightLEDState) {
            case RED:
                rightLEDRedChannel.setState(true);
                rightLEDGreenChannel.setState(false);
                break;
            case GREEN:
                rightLEDRedChannel.setState(false);
                rightLEDGreenChannel.setState(true);
                break;
            case AMBER:
                rightLEDRedChannel.setState(true);
                rightLEDGreenChannel.setState(true);
                break;
            case OFF:
                rightLEDRedChannel.setState(false);
                rightLEDGreenChannel.setState(false);
                break;
        }
    }

    /**
     * Turns off the left and right LED's
     */
    public void turnOff() {
        leftLEDState  = OFF;
        rightLEDState = OFF;
    }

    /**
     * Sets the led state of the left LED (from the back of the robot)
     * @param ledState The {@link org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState}
     *                 to set the left LED to.
     */
    public void setLeftLEDState(@NonNull LEDState ledState) {
        leftLEDState = ledState;
    }

    /**
     * Sets the led state of the right LED (from the back of the robot)
     * @param ledState The {@link org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState}
     *                 to set both of the LED's to.
     */
    public void setRightLEDState(@NonNull LEDState ledState) {
        rightLEDState = ledState;
    }

    /**
     * Sets the led state of the left and right LED's
     * @param ledState The {@link org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState}
     *                 to set both of the LED's to.
     */
    public void setBothLEDStates(@NonNull LEDState ledState) {
        setLeftLEDState(ledState);
        setRightLEDState(ledState);
    }

    /**
     * Displays debug information about the LED Subsystem
     */
    public void debug() {
        telemetry.addLine("----- LED Subsystem Debug -----");
        telemetry.addData("Left LED State", leftLEDState.toString());
        telemetry.addData("Right LED State", rightLEDState.toString());
        telemetry.addData("Left LED Green Channel State", leftLEDGreenChannel.getState());
        telemetry.addData("Left LED Red Channel State", leftLEDRedChannel.getState());
        telemetry.addData("Right LED Green Channel State", rightLEDGreenChannel.getState());
        telemetry.addData("Right LED Red Channel State", rightLEDRedChannel.getState());
    }
}
