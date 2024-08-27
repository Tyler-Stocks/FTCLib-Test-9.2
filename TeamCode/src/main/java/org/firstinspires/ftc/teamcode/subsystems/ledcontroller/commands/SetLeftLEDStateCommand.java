package org.firstinspires.ftc.teamcode.subsystems.ledcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState;
import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDSubsystem;

public class SetLeftLEDStateCommand extends CommandBase {
    private final LEDSubsystem ledSubsystem;
    private final LEDState ledState;

    private boolean isFinished;

    /**
     * Constructs a new SetLeftLEDStateCommand
     * @param ledSubsystem The LED Subsystem to set the left light of
     * @param ledState The state to set the light see
     * {@link org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDState}
     */
    public SetLeftLEDStateCommand(
            @NonNull LEDSubsystem ledSubsystem,
            @NonNull LEDState ledState
    ) {
        this.ledSubsystem = ledSubsystem;
        this.ledState     = ledState;

        isFinished = false;

        addRequirements(ledSubsystem);
    }

    @Override public void execute() {
        ledSubsystem.setLeftLEDState(ledState);
        isFinished = true;
    }

    @Override public boolean isFinished() {
        return isFinished;
    }
}
