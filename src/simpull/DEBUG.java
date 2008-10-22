package simpull;

public final class DEBUG {

	public static String string(Vector2f v) {
		return "x:" +  v.x + ",y:" + v.y + " ";
	}
	
	public static String stringFloat(Vector2f v) {
		return "x:" +  FP.toFloat(v.x) + ",y:" + FP.toFloat(v.y) + " ";
	}
	
}
