package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.transferSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class transferOff extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final transferSubsystem transferSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public transferOff(transferSubsystem subsystem) {
        transferSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

    }


    @Override
    public void execute(){
        transferSubsystem.transferOff();
    }

    @Override
    public boolean isFinished(){
        if (transferSubsystem.getTransferPower() == 0){
            return true;
        } else{
            return false;
        }
    }


}
