package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.finalizedSubsystems.turretSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class shooterOn extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final shooterSubsystem shooterSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public shooterOn(shooterSubsystem subsystem) {
        shooterSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    public double p = 11, i = 0, d = 0, f = 0.8;

    public PIDFController shooterController = new PIDFController(p, i, d, f);

    public double ticksPerDegree = 4.233;
    double target = 1230;


    @Override
    public void initialize(){
        shooterController.setPIDF(p,i,d,f);
    }


    @Override
    public void execute(){

        double shooterVelocity = shooterSubsystem.getShooterVelocity();

        double output  = shooterController.calculate(shooterVelocity, target);

        shooterSubsystem.setShooterPower(output);

    }




    @Override
    public boolean isFinished(){
        return false;
    }


}
