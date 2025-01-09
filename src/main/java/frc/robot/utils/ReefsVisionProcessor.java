package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import frc.lib.vision.VisionProcessor;
import frc.robot.Constants.ReefsVisionConstants;

public class ReefsVisionProcessor extends VisionProcessor {

	public ReefsVisionProcessor() {
		super(
				ReefsVisionConstants.CAMERA_INDEX,
				ReefsVisionConstants.STREAM_NAME,
				ReefsVisionConstants.IMAGE_WIDTH,
				ReefsVisionConstants.IMAGE_HEIGHT);
	}

	@Override
	protected List<Mat> processPipeline(Mat inputMat) {
		List<Mat> processedMats = new ArrayList<>();

		// Converte a imagem para o espaço de cor HSV
		Mat hsvMat = new Mat();
		Imgproc.cvtColor(inputMat, hsvMat, Imgproc.COLOR_BGR2HSV);

		// Define os intervalos para a cor vermelha em HSV
		Scalar lowerRed1 = new Scalar(0, 120, 70); // Faixa 1 do vermelho
		Scalar upperRed1 = new Scalar(10, 255, 255);
		Scalar lowerRed2 = new Scalar(170, 120, 70); // Faixa 2 do vermelho
		Scalar upperRed2 = new Scalar(180, 255, 255);

		// Cria máscaras para os dois intervalos de vermelho
		Mat mask1 = new Mat();
		Mat mask2 = new Mat();
		Core.inRange(hsvMat, lowerRed1, upperRed1, mask1);
		Core.inRange(hsvMat, lowerRed2, upperRed2, mask2);

		// Combina as duas máscaras
		Mat redMask = new Mat();
		Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, redMask);

		// Encontra contornos na máscara resultante
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		// Identifica o maior contorno
		double maxArea = 0;
		Rect largestRect = null;
		
		for (MatOfPoint contour : contours) {
			double area = Imgproc.contourArea(contour);
			if (area > maxArea) {
				maxArea = area;
				largestRect = Imgproc.boundingRect(contour);
			}
		}

		// Desenha o retângulo e o centro do objeto, se encontrado
		if (largestRect != null) {
			// Desenha o retângulo verde ao redor do maior objeto
			Imgproc.rectangle(inputMat, largestRect, new Scalar(0, 255, 0), 3);

			// Calcula o centro do retângulo
			int rectCenterX = largestRect.x + largestRect.width / 2;
			int rectCenterY = largestRect.y + largestRect.height / 2;

			// Desenha um círculo no centro do objeto
			Imgproc.circle(inputMat, new Point(rectCenterX, rectCenterY), 5, new Scalar(255, 255, 255), -1);

			// Calcula o centro da tela
			int frameCenterX = inputMat.cols() / 2;
			int frameCenterY = inputMat.rows() / 2;

			// Calcula o deslocamento
			int offsetX = rectCenterX - frameCenterX;
			int offsetY = rectCenterY - frameCenterY;

			// Adiciona texto indicando o deslocamento
			String offsetText = String.format("Offset: X=%d, Y=%d", offsetX, offsetY);
			Imgproc.putText(
					inputMat,
					offsetText,
					new Point(10, 30), // Posição do texto na imagem
					Imgproc.FONT_HERSHEY_SIMPLEX,
					1.0, // Tamanho da fonte
					new Scalar(0, 255, 0), // Cor do texto (verde)
					2 // Espessura da fonte
			);
		}

		// Adiciona a imagem original processada à lista
		processedMats.add(inputMat); // Imagem original com o retângulo, o centro e o texto desenhados

		// Libera memória das matrizes temporárias
		hsvMat.release();
		mask1.release();
		mask2.release();
		redMask.release();
		hierarchy.release();

		drawCenterCross(inputMat);

		return processedMats;
	}
}
