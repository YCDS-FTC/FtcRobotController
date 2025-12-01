package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.stopperSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class stopperOff extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final stopperSubsystem stopperSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public stopperOff(stopperSubsystem subsystem) {
        stopperSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

    }


    @Override
    public void execute(){
        stopperSubsystem.stopperOff();
    }

    @Override
    public boolean isFinished(){
        if (stopperSubsystem.getStopperPosition() == 47){
            return true;
        } else{
            return false;
        }
    }


}
