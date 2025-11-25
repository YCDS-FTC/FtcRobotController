package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.intakeSubSystem;

/**
 * An example command that uses an example subsystem.
 */
public class intakeOn extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final intakeSubSystem intakeSubSystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public intakeOn(intakeSubSystem subsystem) {
        intakeSubSystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        intakeSubSystem.intakeOn();
    }


    @Override
    public boolean isFinished(){
        return true;
    }


}
