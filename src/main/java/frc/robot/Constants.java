package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

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
    public static final double ROBOT_MASS = 49; // Massa do robô

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
    public static final double wheelDiameterInMeters = Units.inchesToMeters(3.75);

    // Redução para motor de acionamento e ângulo
    public static final double driveGearRatio = 6.75;
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
    public static class Gains {
      // Ganhos PID para o Elevator
      public static final double kP = 0.03; // Proporcional
      public static final double kI = 0.0; // Integral
      public static final double kD = 0.0; // Derivativo
      public static final double kFF = 0.0; // Feed Forward

      public static final double kVelocityFF = 0.001; // Feedforward
    }

    /**
     * IDs e configurações do sistema.
     */
    public static final class HardwareConfig {
      public static final int kLeftMotorId = 15;
      public static final int kRightMotorId = 16;

      public static final double kClosedLoopRate = 0.3;
      public static final int kStallCurrentLimit = 40;

      public static final double kOutputRange = 1.0;
    }
  }

  /**
   * Configurações do sistema de outtake.
   */
  public static final class OuttakeConstants {
    public static class Gains {
      // Ganhos PID para o Outtake
      public static final double kP = 0.45; // 0.31
      public static final double kI = 0.0; // 0.0
      public static final double kD = 0.035; // 0.07

      private static final double kMinUp = 0.135;
      private static final double kMinDown = 0.055;

      // Ganhos de Feedforward
      // public static final double kG = (kMinUp + kMinDown) / 2; // Gravidade 0.0535
      // public static final double kV = (kMinUp - kMinDown) / 2; // Velocidade 0.0175
      public static final double kG = 0.0535; // Gravidade 0.0535
      public static final double kV = 0.0175; // Velocidade 0.0175
      public static final double kS = 0.0; // Tensão estática
      public static final double kA = 0.0; // Aceleração
    }

    /**
     * Restrições do perfil trapezoidal.
     */
    public static final class TrapezoidProfileConstants {
      // public static final double kMaxVelocity = 50.0; // Velocidade máxima (rad/s)
      public static final double kMaxVelocity = 35.0; // Velocidade máxima (rad/s)
      public static final double kMaxAcceleration = 17.5; // Aceleração máxima (rad/s^2) 12.5
      public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity,
          kMaxAcceleration);
    }

    /**
     * Limites e taxas do braço.
     */
    public static final class ArmConfig {
      public static final Angle kMinAngle = Degrees.of(-98.0);
      public static final Angle kMaxAngle = Degrees.of(87.0);
      public static final Angle kMinSafeAngle = Degrees.of(-80.0);
    }

    /**
     * IDs e configurações do sistema.
     */
    public static final class HardwareConfig {
      public static final int kLeftPivotId = 14;
      public static final int kRightPivotId = 17;
      public static final int kOuttakeId = 18;
      public static final int kAbsoluteEncoderId = 0;

      public static final double kOuttakeEncoderOffset = 70;

      public static final double kGearRatio = 1.0 / 20.0;

      public static final int kStallCurrentLimit = 40;
      public static final double kClosedLoopRate = 0.5;
    }
  }

  public static final class ReefsConstants {

    public static enum ReefsScorePose {
      // HORIZONTAL(8, 0),
      // VERTICAL(8, -90),
      // L1(8, 75),
      INITAL(8, -112),
      L2(20, -87),
      L3(55, -90),
      L4(54, 60),
      REMOVEALGAE(4, 10),
      CLIMBPOSE(8,105);

      public final double height;
      public final double angle;

      private ReefsScorePose(double height, double angle) {
        this.height = height;
        this.angle = angle;
      }
    }
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

  /*
   * Mapeamento dos botões POV
   */
  public static final class POV {

    public static final int UP = 0;
    public static final int UP_RIGHT = 45;
    public static final int RIGHT = 90;
    public static final int DOWN_RIGHT = 135;
    public static final int DOWN = 180;
    public static final int DOWN_LEFT = 225;
    public static final int LEFT = 270;
    public static final int UP_LEFT = 315;
  }

  public static final class ClimberMotor {

    public static final int m_climber = 4;
  }

  // public static final class Setpoints {
  //   private static final double armNormal = 0.5;
  //   public static final double armScore = armNormal + 0.37;
  // }
}