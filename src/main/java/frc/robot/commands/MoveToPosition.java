package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosition extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Pose2d targetPose2d;
    private static final double TOLERANCIA = 0.1;

    public MoveToPosition(SwerveSubsystem swerveSubsystem, double x, double y){
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose2d =  new Pose2d(new Translation2d(x,y), swerveSubsystem.getPose().getRotation());
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        
        //Descobre onde o robô ta na arena
        //Discover where the robot is in the arena
        Pose2d posicaoAtual = swerveSubsystem.getPose();

        //Calcula a diferença entre a posição em que o robô está e a posição em que queremos chegar
        //Calculates the difference between the position the robot is in and the position we want to reach
        Translation2d erro = targetPose2d.getTranslation().minus(posicaoAtual.getTranslation());

        //Calcula a distância do erro de cada eixo, que é usado como velocidade para mover o robô
        //Calculates the error distance of each axis, which is used as the speed to move the robot
        double vX = erro.getX();
        double vY = erro.getY();

        //Calcula a velocidade max como a hipotenusa das distância do erro dos pontos
        //Calculate the max speed as the hypotenuse of the error distance of the points
        double velocidadeMax = Math.hypot(vX, vY);


        //É para saber se foi excedido a velocidade max, se foi ela é ajustada para ficar correto
        //It is to know if the max speed has been exceeded, if so it is adjusted to be correct.
        if (velocidadeMax > 1.0) {
            vX /= velocidadeMax;
            vY /= velocidadeMax;
        }

        //Faz o robô andar para a posição desejada
        //Moves the robot to the desired position
        swerveSubsystem.drive(new Translation2d(vX, vY), 0, true);
    }


    //Calcula a diferença entre as posições novamente para saber se é menor que a tolerância
    //Calculates the difference between the positions again to see if it is smaller than the tolerance
    @Override
    public boolean isFinished(){
        Translation2d erro = targetPose2d.getTranslation().minus(swerveSubsystem.getPose().getTranslation());
        return erro.getNorm() < TOLERANCIA;
    }
    
    //Faz o robô parar se estiver na posição
    //O interromper serve para parar o robô caso outro comando seja executado durante o processo
    //Makes the robot stop if it is in position
    //O interrupt is used to stop the robot if another command is executed during the process
    @Override
    public void end(boolean interromper){
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
