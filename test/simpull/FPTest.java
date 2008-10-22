package simpull;

public class FPTest {

	public static void main(String[] args) {
		FPTest fpTest = new FPTest();
		// fpTest.specificTestCase();
		 fpTest.testSquaresLimits();
		// fpTest.testSinCos();
		
		//fpTest.playground();
	}
	
	private void playground() {
		int size = 800;
		long startTime = System.currentTimeMillis();
		System.out.println("starting to build....");
		int [][] c = new int[size][size];
		for (int a = 0; a < size; ++a) {
			for (int b = 0; b < size; ++b) {
				c[a][b] = (int)Math.sqrt(a * a + b * b);
			}
		}
		System.out.println("..finished in " + (System.currentTimeMillis() - startTime) + "ms");
		System.out.println("\nc[799][799] = " + c[799][799]);
		System.out.println("\nMath.sqrt(799.999 * 799.999 + 799.999 * 799.999) = " + Math.sqrt(799.999 * 799.999 + 799.999 * 799.999));
	}
	
	private void testSinCos() {
		float radian = 0;
		while ((radian += 0.1f) < (Math.PI * 2)) {
			int radianFX = FP.from(radian);

			System.out.println("radian: " + radian);
			System.out.println("cos     : " + Math.cos(radian) + "\tsin     : " + Math.sin(radian));
			System.out.println("cos (fx): " + FP.toFloat(FP.cos(radianFX)) +        "\tsin (fx): " + FP.toFloat(FP.sin(radianFX)));
			System.out.println("---------------------------------------------------------------");
		}
	}
	
	private void testSquaresLimits() {
		float value = 0;
		while ((value += 1) < 255f) {
			int valueFX = FP.from(value);
			long valueSquaredFX = (((long) valueFX * valueFX) >> FP.FRACTION_BITS);

			System.out.println("value     : " + value + "\tvalue^2     : " + (value * value) + "\tsqrt(value)     :" + Math.sqrt(value * value));
			System.out.println("value (fx): " + FP.toFloat(valueFX) +        "\tvalue^2 (fx): " + toFloat(valueSquaredFX) + "\tsqrt(value) (fx):" + toFloat(FP.sqrt(valueSquaredFX)));
			System.out.println("---------------------------------------------------------------");
		}
	}
	
	private static float toFloat(long fixedPoint) {
		return (float)fixedPoint / FP.ONE;
	}
	
	private void bob() {
		
	}
	
	private void specificTestCase() {
		int x = 21296378; // fixed point representation of 324.95694
		System.out.println("\nx as fixed: " + x);
		System.out.println("x as float: " + FP.toFloat(x));

		long xLong = x;
		System.out.println("\nxLong as fixed: " + (int)xLong);
		System.out.println("xLong as float: " + FP.toFloat((int)xLong));

		int y = -7219805; // fixed point representation of -110.16548
		System.out.println("\ny as fixed: " + y);
		System.out.println("y as float: " + FP.toFloat(y));
		
		int xSquared = (int) (((long) x * x) >> FP.FRACTION_BITS);
		System.out.println("\nx^2 as fixed: " + xSquared);
		System.out.println("x^2 as float: " + FP.toFloat(xSquared) + "\tthis value should be: "+(324.95694f * 324.95694f));
		
		long xLongSquared = (xLong * xLong) >> FP.FRACTION_BITS;
		System.out.println("\nxLong^2 as fixed: " + (int)xLongSquared);
		System.out.println("xLong^2 as float: " + FP.toFloat((int)xLongSquared) + "\tthis value should be: "+(324.95694f * 324.95694f));
		
		

		int ySquared = (int) (((long) y * y) >> FP.FRACTION_BITS);
		System.out.println("\ny^2 as fixed: " + ySquared);
		System.out.println("y^2 as float: " + FP.toFloat(ySquared) + "\tthis value should be: "+(-110.16548f * -110.16548f));
	}
	
}
