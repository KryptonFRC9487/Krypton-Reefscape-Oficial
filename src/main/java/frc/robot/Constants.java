package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;

/**
 * Classe Constants para armazenar constantes usadas no código do robô.
 * Organizada em classes internas para melhor separação de responsabilidades.
 */
public final class Constants {
  /**
   * Configurações relacionadas às dimensões e características físicas do robô.
   */
  public static final class Dimensoes {
    public static final double LOOP_TIME = 0.13; // Tempo de loop (sparkMax + normal = 130ms)
    public static final double ROBOT_MASS = 38; // Massa do robô

    // Posições do centro de massa
    private static final double X_MASS = 0;
    private static final double Y_MASS = 0;
    private static final double Z_MASS = .08;

    // Centro de massa do chassi
    public static final Matter CHASSIS = new Matter(
        new Translation3d(X_MASS, Y_MASS, (Z_MASS)), ROBOT_MASS);

    // Máxima aceleração e velocidade
    public static final double MAX_ACCE_AUTO = 4;
    public static final double MAX_VEL_AUTO = 4;

    // Diâmetro da roda do módulo
    public static final double wheelDiameterInMeters = Units.inchesToMeters(4);

    // Redução para motor de acionamento e ângulo
    public static final double driveGearRatio = 6.18;
    public static final double angleGearRatio = 21.42;

    // PPR do encoder interno NEO;
    public static final double pulsePerRotation = 1;

    // Fatores de conversão para motores de acionamento e ângulo
    public static final double driveConversion = SwerveMath.calculateMetersPerRotation(wheelDiameterInMeters,
        driveGearRatio, pulsePerRotation);
    public static final double angleConversion = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio,
        pulsePerRotation);
  }

  /**
   * Configurações de PID para controle autônomo.
   */
  public static final class PID {
    public static final PIDFConfig xAutoPID = new PIDFConfig(0.65, 0, 0.05);
    public static final PIDFConfig yAutoPID = new PIDFConfig(0.6, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.1, 0, 0.01);
  }

  /**
   * Configurações relacionadas ao controle de tração.
   */
  public static final class Tracao {
    public static final boolean fieldRelative = true; // Orientação ao campo
    public static final boolean isOpenLoop = false; // Malha fechada
    public static final boolean accelCorrection = false; // Correção de aceleração

    // Multiplicadores para suavizar entradas do joystick
    public static final double multiplicadorRotacional = 0.8;
    public static final double multiplicadorTranslacionalY = 0.7;
    public static final double multiplicadorTranslacionalX = 0.7;

    public static final double TURN_CONSTANT = 0.75; // Constante de rotação
    public static final double MAX_SPEED = 4.8; // Velocidade máxima (m/s)
    public static final double DT = 0.02; // Intervalo de tempo (s)
    public static final double constantRotation = 4.0; // Rotação constante
  }

  /**
   * Configurações das trajetórias autônomas.
   */
  public static final class Trajetoria {
    public static final String NOME_TRAJETORIA = "New Auto";
    public static final String NOME_TRAJETORIA2 = "New Path2";
  }

  /**
   * Estados usados em strings.
   */
  public static final class StateStrings {

    public static final String OFF = "OFF";
    public static final String ON = "ON";
  }

  /**
   * Configurações do elevador.
   */
  public static final class ElevatorConstants {

    public static enum ElevatorPosition {
      BOTTOM(1),
      TOP(10);

      public final double value;

      private ElevatorPosition(double value) {
        this.value = value;
      }
    }

    // IDs dos motores
    public static final int LEFT_MOTOR_ID = 15;
    public static final int RIGHT_MOTOR_ID = 16;

    // Constantes do elevador
    public static final double GEARING = 5.0;
    public static final double MASS_KG = 5.0;
    public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * Units.inchesToMeters(2.0);
    public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

    public static final int CURRENT_LIMIT = 60;

    // PID Gains
    public static final double kP = 0.3; // Proporcional
    public static final double kI = 0.0; // Integral
    public static final double kD = 0.0; // Derivativo
    public static final double kFF = 0.05; // Feedforward

    public static final double kVelocityFF = 0.001; // Feedforward

    // Feedforward Gains
    public static final double kS = 0.095388; // Tensão estática TODO
    public static final double kG = 0.54402; // Gravidade TODO
    public static final double kV = 7.43; // Velocidade TODO
    public static final double kA = 1.0; // Aceleração TODO

    // Restrições de movimento
    public static final double MIN_HEIGHT_METERS = 0.005; // TODO
    public static final double MAX_HEIGHT_METERS = 1.57; // TODO
    public static final double MAX_VELOCITY = 3.67; // Velocidade máxima (m/s) TODO
    public static final double MAX_ACCELERATION = 3.0; // Aceleração máxima (m/s^2) TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY, MAX_ACCELERATION);
  }

  /**
   * Configurações do sistema de outtake.
   */
  public static final class OuttakeConstants {
    public static final int LEFT_PIVOT_ID = 14;
    public static final int RIGHT_PIVOT_ID = 17;
    public static final int OUTTAKE_ID = 18;

    // PID Gains
    public static final double kP = 0.2; // Proporcional 1.35
    public static final double kI = 0.0; // Integral
    public static final double kD = 0.0; // Derivativo

    // Feedforward Gains
    public static final double kS = 0.1; // Tensão estática TODO
    public static final double kG = 1.79; // Gravidade TODO
    public static final double kV = 0.29; // Velocidade TODO
    public static final double kA = 0.05; // Aceleração TODO

    public static final double MAX_VELOCITY = 118.47; // Velocidade máxima (rad/s) TODO
    public static final double MAX_ACCELERATION = 25.0; // Aceleração máxima (rad/s^2) TODO
    public static final TrapezoidProfile.Constraints TRAPEZOID_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY, MAX_ACCELERATION);
  }

  /**
   * Configurações de posição do outtake
   */
  public static final class Outtake {

    public static enum OuttakePose {
      INIT,
      DEPOSIT,
    }

    public static final double OUTTAKE_INIT = 0.40;
    public static final double OUTTAKE_DEPOSIT = 0.55;
  }

  /**
   * Configurações de visão.
   */
  public static final class ReefsVisionConstants {
    public static final int CAMERA_INDEX = 1;
    public static final String STREAM_NAME = "Red Detection";
    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;
  }

  /**
   * Configurações do controle e botões.
   */
  public static final class GamepadConstants {
    public static final int P1_PORT = 0;
    public static final int P2_PORT = 1;
    public static final double DEADBAND = 0.2; // Deadband do controle P1
  }

  public static final class Buttons {
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BUTTON_BACK = 7;
  }
}
