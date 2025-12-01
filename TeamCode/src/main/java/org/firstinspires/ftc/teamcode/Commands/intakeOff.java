package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.intakeSubSystem;

/**
 * An example command that uses an example subsystem.
 */
public class intakeOff extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final intakeSubSystem intakeSubSystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public intakeOff(intakeSubSystem subsystem) {
        intakeSubSystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

    }


    @Override
    public void execute(){
        intakeSubSystem.intakeOff();
    }

    @Override
    public boolean isFinished(){
        if (intakeSubSystem.getIntakePower() == 0){
            return true;
        } else{
            return false;
        }    }


}
