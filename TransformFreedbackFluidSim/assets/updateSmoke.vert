#version 150 core

in vec3 VertexPosition;
in vec3 VertexVelocity;
in float VertexStartTime;
in vec3 VertexInitialVelocity;
in vec4 VertexColor;

out vec3 Position; // To Transform Feedback
out vec3 Velocity; // To Transform Feedback
out vec4 Color; // To Transform Feedback
out float StartTime; // To Transform Feedback


uniform float Time; // Time
uniform float H;	// Elapsed time between frames
uniform vec3 Accel; // Particle Acceleration
uniform float ParticleLifetime; // Particle lifespan

uniform bool MouseIsDown;
uniform vec2 MousePosition;
uniform vec2 MinPosition;
uniform vec2 MaxPosition;

void main() {
	
	// Update position & velocity for next frame
	Position = VertexPosition;
	Velocity = VertexVelocity;
	StartTime = VertexStartTime;

	//Bounce if the velocity is going to bring it outside the imaginary box.
	if (Position.x + (Velocity.x * H) < MinPosition.x || Position.x + (Velocity.x * H) > MaxPosition.x)
		Velocity.x = -Velocity.x;

	if (Position.y + (Velocity.y * H) < MinPosition.y || Position.y + (Velocity.y * H) > MaxPosition.y)
		Velocity.y = -Velocity.y;
	
	Position += Velocity * H;
	Velocity += Accel * H;
	if( MouseIsDown )
	{
		float dist = distance(MousePosition, vec2(Position.x, Position.y));
		vec2 dir = vec2(Position.x, Position.y) - MousePosition;

		float mouseStrength = 0.5; //higher adds more velocity
		vec2 v = normalize(dir) * mouseStrength / dist;
		Velocity += vec3(v.x, v.y, 0);
	}
}