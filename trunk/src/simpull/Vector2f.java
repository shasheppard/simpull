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
	
/** TODO remove most methods from here for performance reasons.
 * Almost all references to these methods have already been removed.
 * The only remaining class to refactor is {@link BaseCollection}.
 */
public final strictfp class Vector2f {
	
	public float x;
	public float y;

	public Vector2f() {
		x = 0f;
		y = 0f;
	}
	
	public Vector2f(float x, float y) {
		this.x = x;
		this.y = y;
	}

	public float dot(Vector2f vector) {
		return x * vector.x + y * vector.y;
	}
	
	public float cross(Vector2f v) {
		return x * v.y - y * v.x;
	}
	
	public Vector2f plus(Vector2f v) {
		return new Vector2f(x + v.x, y + v.y); 
	}

	public Vector2f plusEquals(Vector2f v) {
		x += v.x;
		y += v.y;
		return this;
	}
	
	public Vector2f minus(Vector2f v) {
		return new Vector2f(x - v.x, y - v.y);    
	}

	public Vector2f minusEquals(Vector2f v) {
		x -= v.x;
		y -= v.y;
		return this;
	}

	public Vector2f mult(float s) {
		return new Vector2f(x * s, y * s);
	}

	public Vector2f multEquals(float s) {
		x *= s;
		y *= s;
		return this;
	}

	public Vector2f times(Vector2f v) {
		return new Vector2f(x * v.x, y * v.y);
	}

	public Vector2f divEquals(float s) {
		if (s == 0) {
			s = 0.0001f;
		}
		x /= s;
		y /= s;
		return this;
	}

	public float magnitude() {
		return (float)Math.sqrt(x * x + y * y);
	}

	public float distance(Vector2f v) {
		Vector2f delta = this.minus(v);
		return delta.magnitude();
	}

	public Vector2f normalize() {
		 float magnitude = (float)Math.sqrt(x * x + y * y);
		 if (magnitude == 0) {
			 magnitude = 0.0001f;
		 }
		 float invMagnitude = 1 / magnitude;
		 Vector2f normalized = new Vector2f(x, y);
		 normalized.x *= invMagnitude;
		 normalized.y *= invMagnitude;
		 return normalized;
	}
	
	public Vector2f normalizeEquals() {
		 float magnitude = (float)Math.sqrt(x * x + y * y);
		 if (magnitude == 0) {
			 magnitude = 0.0001f;
		 }
		 float invMagnitude = 1 / magnitude;
		 x *= invMagnitude;
		 y *= invMagnitude;
		 return this;
	}
		
}