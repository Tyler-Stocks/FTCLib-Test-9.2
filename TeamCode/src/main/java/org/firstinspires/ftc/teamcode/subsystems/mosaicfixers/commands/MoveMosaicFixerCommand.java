package org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerPosition;
import org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.utility.RobotSide;

public class MoveMosaicFixerCommand extends CommandBase {
    private final MosaicFixerSubsystem mosaicFixerSubsystem;
    private final MosaicFixerPosition  mosaicFixerPosition;
    private final RobotSide            mosaicFixerSide;

    private boolean isFinished;

    /**
     * Constructs a new MoveMosaicFixerCommand
     * @param mosaicFixerSubsystem The subsystem to move the arm of
     * @param mosaicFixerPosition The position to move the arm to
     * @param mosaicFixerSide Which arm to move
     */
    public MoveMosaicFixerCommand(
            @NonNull MosaicFixerSubsystem mosaicFixerSubsystem,
            @NonNull MosaicFixerPosition  mosaicFixerPosition,
            @NonNull RobotSide            mosaicFixerSide
    ) {
        this.mosaicFixerSubsystem = mosaicFixerSubsystem;
        this.mosaicFixerPosition  = mosaicFixerPosition;
        this.mosaicFixerSide      = mosaicFixerSide;

        isFinished = false;

        addRequirements(mosaicFixerSubsystem);
    }

    @Override public void execute() {
        switch (mosaicFixerSide) {
            case LEFT:
                mosaicFixerSubsystem.enableLeftMosaicFixer();
                mosaicFixerSubsystem.moveLeftMosaicFixerToPosition(mosaicFixerPosition);
                break;
            case RIGHT:
                mosaicFixerSubsystem.enableRightMosaicFixer();
                mosaicFixerSubsystem.moveRightMosaicFixerToPosition(mosaicFixerPosition);
                break;
            case BOTH:
                mosaicFixerSubsystem.enableLeftMosaicFixer();
                mosaicFixerSubsystem.enableRightMosaicFixer();
                mosaicFixerSubsystem.moveLeftMosaicFixerToPosition(mosaicFixerPosition);
                mosaicFixerSubsystem.moveRightMosaicFixerToPosition(mosaicFixerPosition);
        }
        isFinished = true;
    }

    @Override public boolean isFinished() {
        return isFinished;
    }
}
