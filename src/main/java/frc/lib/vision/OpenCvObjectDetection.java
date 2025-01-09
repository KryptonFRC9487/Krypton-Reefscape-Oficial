package frc.lib.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * A classe VisionProcessor é responsável pelo processamento de vídeo utilizando
 * o OpenCV, uma biblioteca de visão computacional de código aberto.
 * 
 * O OpenCV usa a convenção de cores <b>BGR (Blue, Green, Red)</b>, onde os
 * valores de cor devem ser passados na ordem azul (B), verde (G) e vermelho
 * (R).
 * 
 * Por exemplo, para criar a cor vermelha, usa-se {@code new Scalar(0, 0, 255)}.
 * 
 * A classe captura o fluxo de vídeo e permite o processamento personalizado
 * através do método abstrato {@link #processPipeline(Mat)}.
 */
public abstract class OpenCvObjectDetection {
	private UsbCamera camera;
	private CvSink cvSink;
	private CvSource outputStream;
	private Mat mat;

	/**
	 * Inicializa a câmera com o índice fornecido e configura a resolução.
	 * Configura também o CvSink (para capturar o vídeo) e o CvSource (para exibir o
	 * vídeo).
	 *
	 * @param cameraIndex O índice da câmera a ser utilizada.
	 * @param streamName  O nome do stream de vídeo.
	 * @param width       Largura da resolução da câmera.
	 * @param height      Altura da resolução da câmera.
	 */
	public OpenCvObjectDetection(int cameraIndex, String streamName, int width, int height) {
		initializeCamera(cameraIndex, streamName, width, height);

		mat = new Mat();
	}

	/**
	 * Inicializa a câmera com o índice fornecido e configura a resolução.
	 * Configura também o CvSink (para capturar o vídeo) e o CvSource (para exibir o
	 * vídeo).
	 *
	 * @param cameraIndex O índice da câmera a ser utilizada.
	 * @param streamName  O nome do stream de vídeo.
	 * @param width       Largura da resolução da câmera.
	 * @param height      Altura da resolução da câmera.
	 */
	private void initializeCamera(int cameraIndex, String streamName, int width, int height) {
		camera = CameraServer.startAutomaticCapture(cameraIndex);
		camera.setResolution(width, height);

		cvSink = CameraServer.getVideo();
		outputStream = CameraServer.putVideo(streamName, width, height);
	}

	/**
	 * Inicia a captura e o processamento do vídeo em uma nova thread.
	 * A thread chama o método processFrame continuamente até ser interrompida.
	 */
	public void start() {
		Thread visionThread = new Thread(() -> {
			while (!Thread.interrupted()) {
				processFrame();
			}
		});

		visionThread.setDaemon(true);
		visionThread.start();
	}

	/**
	 * Processa cada frame de vídeo capturado. Este método chama o pipeline
	 * específico de visão
	 * que é implementado na subclasse.
	 */
	private void processFrame() {
		try {
			if (cvSink.grabFrame(mat) == 0 || mat.empty()) {
				outputStream.notifyError(cvSink.getError());
				return;
			}

			List<Mat> processedMats = processPipeline(mat);

			if (!processedMats.isEmpty()) {
				outputStream.putFrame(processedMats.get(0));
			}

		} catch (Exception e) {
			throw e;
		}
	}

	/**
	 * Método abstrato que deve ser implementado pela subclasse para realizar o
	 * pipeline de processamento.
	 * A subclasse deve definir o processamento específico que será feito no frame
	 * de vídeo.
	 *
	 * @param inputMat O frame de vídeo capturado.
	 * @return Uma lista de imagens processadas.
	 */
	protected abstract List<Mat> processPipeline(Mat inputMat);

	/**
	 * Desenha um "X" no centro da imagem.
	 * Este método pode ser utilizado para marcar a posição central da imagem.
	 *
	 * @param mat A imagem onde o "X" será desenhado.
	 */
	protected void drawCenterCross(Mat mat) {
		int frameWidth = mat.cols();
		int frameHeight = mat.rows();
		int centerX = frameWidth / 2;
		int centerY = frameHeight / 2;

		Imgproc.circle(mat, new Point(centerX, centerY), 5, new Scalar(255, 255, 255), -1);
	}
}
