package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.intakeSubSystem;

/**
 * An example command that uses an example subsystem.
 */
public class intakeLow extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final intakeSubSystem intakeSubSystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public intakeLow(intakeSubSystem subsystem) {
        intakeSubSystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        intakeSubSystem.intakeLow();
    }


    @Override
    public boolean isFinished(){
        return true;
    }


}
