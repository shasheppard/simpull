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
	
final strictfp class MathUtil {

	public static final float PI = (float)Math.PI;
	public static final float TWO_PI = 2f * (float)Math.PI;
	
	private static final Vector2f tmp = new Vector2f();
	
	/** @return n clamped between min and max */	
	static float clamp(float n, float min, float max) {
		if (n < min) {
			return min;
		}
		if (n > max) {
			return max;
		}
		return n;
	}
	
	/** @return 1 if the value is >= 0. Returns -1 if the value is < 0. */	
	static int sign(float val) {
		if (val < 0) {
			return -1;
		}
		return 1;
	}
	
	/*
	static float getThetaB(Vector2f a, Vector2f b, Vector2f c) {
		tmp.x = b.x - c.x;
		tmp.y = b.y - c.y;
		float hypotenuse = (float)Math.sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
		tmp.x = a.x - b.x;
		tmp.y = a.y - b.y;
		float adjacent = (float)Math.sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
		float theta;
		if (hypotenuse == 0f) { // avoid division by zero
			tmp.x = a.x - c.x;
			tmp.y = a.y - c.y;
			float opposite = (float)Math.sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
			theta = (float)Math.atan(opposite / adjacent);
		} else {
			theta = (float)Math.acos(adjacent / hypotenuse);
		}
		// TODO change theta depending on the location of a
		return theta;
	}
	*/
	
}