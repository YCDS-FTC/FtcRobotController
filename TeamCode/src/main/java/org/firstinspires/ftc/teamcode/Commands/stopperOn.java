package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.stopperSubsystem;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.transferSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class stopperOn extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final stopperSubsystem stopperSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public stopperOn(stopperSubsystem subsystem) {
        stopperSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        stopperSubsystem.stopperOn();
    }



    @Override
    public boolean isFinished(){
        if (stopperSubsystem.getStopperPosition() == 67){
            return true;
        } else{
            return false;
        }
    }


}
