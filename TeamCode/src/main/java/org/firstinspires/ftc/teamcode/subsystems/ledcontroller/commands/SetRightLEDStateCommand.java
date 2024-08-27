package org.firstinspires.ftc.teamcode.subsystems.ledcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState;
import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDSubsystem;

public class SetRightLEDStateCommand extends CommandBase {
    private final LEDSubsystem ledSubsystem;
    private final LEDState ledState;

    private boolean isFinished;

    /**
     * Constructs a new SetRightLEDStateCommand
     * @param ledSubsystem The LED Subsystem to set the right LED state of
     * @param ledState The LED state to set the right LED to.
     */
    public SetRightLEDStateCommand(
            @NonNull LEDSubsystem ledSubsystem,
            @NonNull LEDState ledState
    ) {
        this.ledSubsystem = ledSubsystem;
        this.ledState     = ledState;

        isFinished = false;

        addRequirements(ledSubsystem);
    }

    @Override public void execute() {
        ledSubsystem.setRightLEDState(ledState);
        isFinished = true;
    }

    @Override public boolean isFinished() {
        return isFinished;
    }
}
