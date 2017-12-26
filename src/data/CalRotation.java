package data;

import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;

import display.Assets;

public class CalRotation {

	public static AffineTransformOp rollPointer(float roll) {
		double rotationRequired = Math.toRadians (roll);
		double locationX = Assets.roll.getWidth()/2;
		double locationY = Assets.roll.getHeight()/2;
		AffineTransform tx = AffineTransform.getRotateInstance(rotationRequired, locationX, locationY);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		return op;
	}
	
	public static AffineTransformOp pfdMain(float roll, float pitch) {
		double rotationRequired = Math.toRadians (roll);
		double locationX = Assets.pfdMain.getWidth()/2;
		double locationY = Assets.pfdMain.getHeight()/2 - pitch * 5;
		AffineTransform tx = AffineTransform.getRotateInstance(rotationRequired, locationX, locationY);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		return op;
	}
	
	public static AffineTransformOp headingPointer(float heading) {
		double rotationRequired = Math.toRadians (heading);
		double locationX = Assets.heading.getWidth()/2;
		double locationY = Assets.heading.getHeight()/2;
		AffineTransform tx = AffineTransform.getRotateInstance(rotationRequired, locationX, locationY);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		return op;
	}
}
