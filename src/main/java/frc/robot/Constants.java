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

    // IDs dos motores
    public static final int LEFT_MOTOR_ID = 15;
    public static final int RIGHT_MOTOR_ID = 16;

    // Constantes do elevador
    public static final double GEARING = 5.0;
    public static final double MASS_KG = 5.0;
    public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * Units.inchesToMeters(2.0);
    public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

    public static final int CURRENT_LIMIT = 60;

    // PID Gains Elevator
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
    public static final TrapezoidProfile.Constraints MOVEMENT_pCONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_VELOCITY, MAX_ACCELERATION);
  }

  /**
   * Configurações do sistema de outtake.
   */
  public static final class OuttakeConstants {
    public static class Gains {
      // Ganhos PID para o Outtake
      public static final double kP = 0.0; // 0.0.17
      public static final double kI = 0.0; // 0.0
      public static final double kD = 0.0; // 0.015

      private static final double kMinUp = 0.0; // motor.set() 0.071
      private static final double kMinDown = 0.0; // // motor.set() 0.036

      // Ganhos de Feedforward
      public static final double kG = (kMinUp + kMinDown) / 2; // Gravidade 0.0535
      public static final double kV = (kMinUp - kMinDown) / 2; // Velocidade 0.0175
      public static final double kS = 0.0; // Tensão estática
      public static final double kA = 0.0; // Aceleração
    }

    /**
     * Restrições do perfil trapezoidal.
     */
    public static final class TrapezoidProfileConstants {
      public static final double kMaxVelocity = 50.0; // Velocidade máxima (rad/s)
      public static final double kMaxAcceleration = 25.0; // Aceleração máxima (rad/s^2)
      public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity,
          kMaxAcceleration);
    }

    /**
     * Limites e taxas do braço.
     */
    public static final class ArmConfig {
      public static final int kStallCurrentLimit = 40;
      public static final double kClosedLoopRate = 0.5;
      public static final Angle kMinAngle = Degrees.of(-98.0);
      public static final Angle kMaxAngle = Degrees.of(87.0);
      public static final Angle kMinSafeAngle = Degrees.of(-45.0);
    }

    /**
     * IDs e configurações do sistema.
     */
    public static final class HardwareConfig {
      public static final int kLeftPivotId = 14;
      public static final int kRightPivotId = 17;
      public static final int kOuttakeId = 18;
      public static final int kAbsoluteEncoderId = 0;

      public static final double kOuttakeEncoderOffset = 129.0;

      public static final double kGearRatio = 1.0 / 20.0;
    }
  }

  public static final class ReefsConstants {

    public static enum ReefsScorePose {
      INITAL(2, -100),
      L1(2, 75),
      L2(11, -80),
      L3(2, 69),
      L4(57, 76);

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
}