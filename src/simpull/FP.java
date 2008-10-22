/*
    Copyright (c) 2008, Shaun Curtis Sheppard
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright 
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright 
          notice, this list of conditions and the following disclaimer in the 
          documentation and/or other materials provided with the distribution.
        * Neither the name of Shaun Curtis Sheppard nor the names of its 
          contributors may be used to endorse or promote products derived from 
          this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE.
*/

package simpull;

/** 
 * Fixed point conversion (to/from) utility.
 * It also includes some utilities for math.
 * This is really a replacement for MathUtil.
 * The change of name is to strengthen the thinking that this is all Fixed-Point, 
 * not to mention the fewer key stroke class name.
 */
public final class FP {
	
    public static final int FRACTION_BITS = 16;
    public static final int FRACTION_MASK = (1 << FRACTION_BITS) - 1;
    public static final int ONE = (1 << FRACTION_BITS);
    public static final int TWO = from(2); // FIXME this can probably be replaced by bitshift by 1 for division by 2
	public static final int POINT_0001 = from(0.0001f);
	public static final int POINT_000001 = from(0.000001f);
    public static final int ONE_HALF = ONE >> 1;
    public static final int MAX_VALUE = (1 << 31) - 1;
    public static final int MIN_VALUE = -(1 << 31);
    /** The maximum integer value that a 32-bit fixed-point value can represent. */
    public static final int MAX_INT_VALUE = (1 << (31 - FRACTION_BITS)) - 1;
    /** The minimum integer value that a 32-bit fixed-point value can represent. */
    public static final int MIN_INT_VALUE = -(1 << (31 - FRACTION_BITS));
    /** The maximum 32-bit floating-point value that a 32-bit fixed-point value can represent. */
    // Math.round(MAX_FLOAT_VALUE * ONE) = MAX_VALUE
    public static final float MAX_FLOAT_VALUE = 32768f; 
    /** The minimum 32-bit floating-point value that a 32-bit fixed-point value can represent. */
    public static final float MIN_FLOAT_VALUE = -32768f;
    /** The maximum 64-bit floating-point value that a 32-bit fixed-point value can represent. */
    // David Brackeen found this by trial and error
    // Math.round(MAX_DOUBLE_VALUE * ONE) = MAX_VALUE
    public static final double MAX_DOUBLE_VALUE = 32767.999992370602;
    /** The minimum 64-bit floating-point value that a 32-bit fixed-point value can represent. */
    public static final double MIN_DOUBLE_VALUE = -32768.0;
    
    public static final int PI = (int)Math.round(Math.PI * ONE);
    public static final int TWO_PI = (int)Math.round(2 * Math.PI * ONE);
    public static final int ONE_HALF_PI = (int)Math.round(.5 * Math.PI * ONE);
    public static final int ONE_HALF_PI_MINUS_TWO_PI = ONE_HALF_PI - TWO_PI;
    public static final int E = (int)Math.round(Math.E * ONE);
    
    private static final int PYTHAGOREAN_SZE = 850;
    public static final int[][] pythagorean = new int[PYTHAGOREAN_SZE][PYTHAGOREAN_SZE];
    static {
		for (int a = 0; a < PYTHAGOREAN_SZE; ++a) {
			for (int b = 0; b < PYTHAGOREAN_SZE; ++b) {
				pythagorean[a][b] = (int)Math.sqrt(a * a + b * b);
			}
		}
    }
    
    /** Number of fractional bits used for some internal calculations */
    private static final int INTERNAL_BITS = 24;
    private static final int INTERNAL_BITS_MINUS_FRACTION_BITS = INTERNAL_BITS - FRACTION_BITS;

    /** For more accurate results for sine/cosine. This number was found by trial and error. */
    private static final int TWO_PI_ERROR_FACTOR = 6;
    private static final int TWO_PI_ERROR =
        (int)Math.round((2*Math.PI*(1 << TWO_PI_ERROR_FACTOR) - 
        (double)(TWO_PI << TWO_PI_ERROR_FACTOR) / ONE) * ONE);
    
    /** Converts an integer to a fixed-point value. */
	public static int from(int integer) {
        if (integer > MAX_INT_VALUE) {
            return MAX_VALUE;
        }
        else if (integer < MIN_INT_VALUE) {
            return MIN_VALUE;
        }
        else {
            return integer << FRACTION_BITS;
        }
	}
	
    /** Converts an integer to a fixed-point value. */
	public static int from(float floatingPoint) {
        if (floatingPoint > MAX_FLOAT_VALUE) {
            return MAX_VALUE;
        }
        else if (floatingPoint < MIN_FLOAT_VALUE) {
            return MIN_VALUE;
        }
        else {
            return Math.round(floatingPoint * ONE);
        }
	}
	
    /** Converts an integer to a fixed-point value. */
	public static int from(double doublePrecision) {
        if (doublePrecision > MAX_DOUBLE_VALUE) {
            return MAX_VALUE;
        }
        else if (doublePrecision < MIN_DOUBLE_VALUE) {
            return MIN_VALUE;
        }
        else {
            return (int)Math.round(doublePrecision * ONE);
        }
	}
	
    /** Converts a fixed-point value to an integer. */
	public static int toInt(int fixedPoint) {
        if (fixedPoint < 0) {
            return toIntCeil(fixedPoint);
        }
        else {
            return toIntFloor(fixedPoint);
        }
	}
	
    /** Converts a fixed-point value to an integer, flooring the result. */
	public static final int toIntFloor(int fixedPoint) {
	    return fixedPoint >> FRACTION_BITS;
	}
	
	/** Converts a fixed-point value to an integer, rounding the result. */
	public static final int toIntRound(int fixedPoint) {
	    return toIntFloor(fixedPoint + ONE_HALF);
	}
	
	/** Converts a fixed-point value to an integer, ceiling the result. */
	public static final int toIntCeil(int fixedPoint) {
	    return -toIntFloor(-fixedPoint);
	}
	
    /** Converts a fixed-point value to a float. */
	public static float toFloat(int fixedPoint) {
        return (float)fixedPoint / ONE;
	}

	public static long toLong(int fixedPoint) {
        if (fixedPoint < 0) {
            return (long)toIntCeil(fixedPoint);
        }
        else {
            return (long)toIntFloor(fixedPoint);
        }
	}

	public static double toDouble(int fixedPoint) {
        return (double)fixedPoint / ONE;
	}
	

    ///////////////////////////////////////////////////////////////////////////
    // Start Fixed-Point trigonometry
    ///////////////////////////////////////////////////////////////////////////
    
    /** @return the sine of the specified fixed-point radian value. */
    public static final int sin(int fixedPoint) {
        if (fixedPoint == 0) {
            return 0;
        }

        // reduce range to -2*pi and 2*pi
        int s = fixedPoint / TWO_PI;
        if (abs(s) >= (1<<TWO_PI_ERROR_FACTOR)) {
            // fix any error for large values of fx
            fixedPoint -= s*TWO_PI + (s >> TWO_PI_ERROR_FACTOR) * TWO_PI_ERROR;
        }
        else {
            fixedPoint -= s*TWO_PI;
        }
        
        // reduce range to -pi/2 and pi/2
        // this allows us to limit the number of iterations in the maclaurin series
        if (fixedPoint > PI) {
            fixedPoint = fixedPoint - TWO_PI;
        }
        else if (fixedPoint < -PI) {
            fixedPoint = fixedPoint + TWO_PI;
        }
        if (fixedPoint > ONE_HALF_PI) {
            fixedPoint = PI - fixedPoint;
        }
        else if (fixedPoint < -ONE_HALF_PI) {
            fixedPoint = -PI - fixedPoint;
        }
        
        // Helps with rotation appearance near 90, 180, 270, 360, etc.
        if (abs(fixedPoint) < 32) {
            return 0;
        } else if (abs(fixedPoint - ONE_HALF_PI) < 32) {
            return ONE;
        } else if (abs(fixedPoint + ONE_HALF_PI) < 32) {
            return -ONE;
        }
        
        // Maclaurin power series
        int fxSquared = (int)(((long)fixedPoint * fixedPoint) >> FRACTION_BITS);
        // above replaces int fxSquared = (int)(((long)fixedPoint * fixedPoint) >> FRACTION_BITS);

        int d = (int)(((long)((1 << INTERNAL_BITS) / (2*3*4*5*6*7*8*9)) * fxSquared) >> FRACTION_BITS);
        // above replaces int d = mul((1 << INTERNAL_BITS) / (2*3*4*5*6*7*8*9), fxSquared);

        int c = (int) (((long) (d - (1 << INTERNAL_BITS) / (2*3*4*5*6*7)) * fxSquared) >> FRACTION_BITS);
        // above replaces int c = mul(d - (1 << INTERNAL_BITS) / (2*3*4*5*6*7), fxSquared);
        
        int b = (int) (((long) (c + (1 << INTERNAL_BITS) / (2*3*4*5)) * fxSquared) >> FRACTION_BITS);
        // above replaced int b = mul(c + (1 << INTERNAL_BITS) / (2*3*4*5), fxSquared);
        
        int a = (int) (((long) (b - (1 << INTERNAL_BITS) / (2*3)) * fxSquared) >> FRACTION_BITS);
        // above replaces int a = mul(b - (1 << INTERNAL_BITS) / (2*3), fxSquared);
        
        int sine = (int) (((long) (a + (1 << INTERNAL_BITS)) * fixedPoint) >> FRACTION_BITS);
        // above replaces int sine = mul(a + (1 << INTERNAL_BITS), fixedPoint);
        
        return sine >> INTERNAL_BITS_MINUS_FRACTION_BITS;
    }
    
    /** @return the cosine of the specified fixed-point radian value. */
    public static final int cos(int fixedPoint) {
        if (fixedPoint == 0) {
            return ONE;
        } else if (fixedPoint < 0) {
            // make up for potential overflow
            return sin(ONE_HALF_PI_MINUS_TWO_PI - fixedPoint);  
        } else {
            return sin(ONE_HALF_PI - fixedPoint);  
        }
    }
    
    /** @return the tangent of the specified fixed-point radian value. */
    public static final int tan(int fixedPoint) {
        int cos = cos(fixedPoint);
        if (cos == 0) {
            return Integer.MAX_VALUE;
        } else {
        	return (int) (((long) sin(fixedPoint) << FRACTION_BITS) / cos);
            // above replaces return div(sin(fx), cos);
        }
    }
    
    /** @return the cotangent of the specified fixed-point radian value. */
    public static final int cot(int fixedPoint) {
        int sin = sin(fixedPoint);
        if (sin == 0) {
            return Integer.MAX_VALUE;
        } else {
        	return (int) (((long) cos(fixedPoint) << FRACTION_BITS) / sin);
            // above replaces return div(cos(fixedPoint), sin);
        }
    }
    
    /** @return the arcsine of the specified fixed-point value. */
    public static final int asin(int fx) {
        if (abs(fx) > ONE) {
            throw new ArithmeticException("NaN");
        } else if (fx == ONE) {
            return ONE_HALF_PI;
        } else if (fx == -ONE) {
            return -ONE_HALF_PI;
        } else {
        	return atan((int) (((long) fx << FRACTION_BITS) / sqrt(ONE - (int) (((long) fx * fx) >> FRACTION_BITS))));
            // above replaces return atan(div(fx, sqrt(ONE - mul(fx, fx))));
        }
    }
    
    /** @return the arccosine of the specified fixed-point value. */
    public static final int acos(int fx) {
        return ONE_HALF_PI - asin(fx);
    }

    /** @return the arctangent of the specified fixed-point value. */
    public static final int atan(int fixedPoint) {
        boolean negative = false;
        boolean invert = false;
        if (fixedPoint == 0) {
            return 0;
        }
        if (fixedPoint < 0) {
            negative = true;
            fixedPoint = -fixedPoint;
        }
        
        // Avoid overflow
        if (fixedPoint > ONE) {
            invert = true;
            fixedPoint = (int) (((long) ONE << FRACTION_BITS) / fixedPoint);
            // above replaces fixedPoint = div(ONE, fixedPoint);
        }
        
        // Approximation from Ranko at http://www.lightsoft.co.uk/PD/stu/stuchat37.html
        // r(x) = (x + 0.43157974*x^3)/(1 + 0.76443945*x^2 + 0.05831938*x^4)
        
        int fxPow2 = (int) (((long) fixedPoint * fixedPoint) >> FRACTION_BITS);
        // above replaces int fxPow2 = mul(fixedPoint, fixedPoint);
        
        int fxPow3 = (int) (((long) fxPow2 * fixedPoint) >> FRACTION_BITS);
        // above replaces int fxPow3 = mul(fxPow2, fixedPoint);
        
        int fxPow4 = (int) (((long) fxPow3 * fixedPoint) >> FRACTION_BITS);
        // above replaces int fxPow4 = mul(fxPow3, fixedPoint);
        
        int numer = fixedPoint + (int) (((long) 28284 * fxPow3) >> FRACTION_BITS);
        // above replaces int numer = fixedPoint + mul(28284, fxPow3);
        
        int denom = ONE + (int) (((long) 50098 * fxPow2) >> FRACTION_BITS) + (int) (((long) 3822 * fxPow4) >> FRACTION_BITS);
        // above replaces int denom = ONE + mul(50098, fxPow2) + mul(3822, fxPow4);
        
        int answer = (int) (((long) numer << FRACTION_BITS) / denom);
        // above replaces int answer = div(numer, denom);
        
        if (invert) {
            answer = ONE_HALF_PI - answer;
        }
        if (negative) {
            answer = -answer;
        }
        return answer;
        
    }
    
    /** @return in the range from -pi to pi. */
    public static final int atan2(int fy, int fx) {
        if (fy == 0) {
            if (fx < 0) {
                return PI;
            } else {
                return 0;
            }
        }
        else if (fx == 0) {
            if (fy < 0) {
                return -ONE_HALF_PI;
            } else {
                return ONE_HALF_PI;
            }
        }
        else {
        	int answer = atan(abs((int) (((long) fy << FRACTION_BITS) / fx)));
            // above replaces int answer = atan(abs(div(fy, fx)));
            if (fy > 0 && fx < 0) {
                return PI - answer;
            } else if (fy < 0 && fx < 0) {
                return answer - PI;
            } else if (fy < 0 && fx > 0) {
                return -answer;
            } else {
                return answer;
            }
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////
    // End of Fixed-Point trigonometry
    ///////////////////////////////////////////////////////////////////////////
    

    ///////////////////////////////////////////////////////////////////////////
    // Start Logs and powers
    ///////////////////////////////////////////////////////////////////////////
    
    public static final int sqrt(int fixedPoint) {
        if (fixedPoint < 0) {
            throw new ArithmeticException("NaN");
        }
        
        if (fixedPoint == 0 || fixedPoint == ONE) {
            return fixedPoint;
        }
        
        // invert numbers less than one (if they aren't too small)
        boolean invert = false;
        if (fixedPoint < ONE && fixedPoint > 6) {
            invert = true;
            fixedPoint = (int) (((long) ONE << FRACTION_BITS) / fixedPoint);
            //above replaces fixedPoint = div(ONE, fixedPoint);
        }
        
        int iterations = 16;
        if (fixedPoint > ONE) {
            // number of iterations == (number of bits in number) / 2
            int s = fixedPoint;
            iterations = 0;
            while (s > 0) {
                s >>=2;
                ++iterations;
            }            
        }
        
        // Newton's iteration
        int l = (fixedPoint >> 1) + 1;
        for (int i=1; i<iterations; i++) {
        	l = (l + (int) (((long) fixedPoint << FRACTION_BITS) / l)) >> 1;
            // above replaces l = (l + div(fixedPoint, l)) >> 1;
        }
        
        // undo the inversion
        if (invert) {
        	return (int) (((long) ONE << FRACTION_BITS) / l);
            // above replaces return div(ONE, l);
        }

        return l;
    }
    
    
    public static final long sqrt(long fixedPoint) {
        if (fixedPoint < 0) {
            throw new ArithmeticException("NaN");
        }
        
        if (fixedPoint == 0 || fixedPoint == ONE) {
            return fixedPoint;
        }
        
        // Invert numbers less than one (if they aren't too small)
        boolean invert = false;
        if (fixedPoint < ONE && fixedPoint > 6) {
            invert = true;
            fixedPoint = (int) (((long) ONE << FRACTION_BITS) / fixedPoint);
            // above replaces fixedPoint = div(ONE, fixedPoint);
        }
        
        int iterations = 16;
        if (fixedPoint > ONE) {
            // Number of iterations == (number of bits in number) / 2
            long s = fixedPoint;
            iterations = 0;
            while (s > 0) {
                s >>= 2;
                iterations++;
            }            
        }
        
        // Newton's iteration
        long l = (fixedPoint >> 1) + 1;
        for (int i=1; i<iterations; i++) {
        	l = (l + (int) (((long) fixedPoint << FRACTION_BITS) / l)) >> 1;
            // above replaces l = (l + div(fixedPoint, l)) >> 1;
        }
        
        // Undo the inversion
        if (invert) {
        	return (int) (((long) ONE << FRACTION_BITS) / l);
            // above replaces return div(ONE, l);
        }

        return l;
    }
    
    public static long dist(int x1, int y1, int x2, int y2) {
        long dx = x1 - x2;
        long dy = y1 - y2;
        return sqrt((int) (((long) dx * dx) >> FRACTION_BITS) + (int) (((long) dy * dy) >> FRACTION_BITS));
        // above replaces return sqrt(mul(dx, dx) + mul(dy, dy));
    }
    
    ///////////////////////////////////////////////////////////////////////////
    // End of Logs and powers
    ///////////////////////////////////////////////////////////////////////////

    
	/** @return value clamped between min and max */	
	static int clamp(int value, int min, int max) {
		if (value < min) {
			return min;
		}
		if (value > max) {
			return max;
		}
		return value;
	}
	
	/** @return 1 if the value is >= 0. Returns -1 if the value is < 0. */	
	static int sign(int value) {
        return (value > 0)?1:((value < 0)?-1:0);
	}
	
    /** @return the absolute value of a number. */
	static int abs(int value) {
	    return (value >= 0)?value:-value;
	}
	
	/** This is a completely static class */
	private FP () {}
	
}
