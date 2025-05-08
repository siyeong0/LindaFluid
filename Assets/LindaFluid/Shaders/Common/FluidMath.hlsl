#ifndef FLUID_MATH_INCLUDE
#define FLUID_MATH_INCLUDE

static const float EPSILON = 0.001f;
static const float2 GRAVITY = float2(0.0, -9.8);
static const float PI = 3.14159265358979323846;

float spikyPow2Kernel(float dist, float radius)
{
	const float C = 6.0 / (PI * pow(radius, 4.0));
	float value = max(0.0, radius - dist);
	return C * pow(value, 2.0);
}

float2 spikyPow2Gradient(float2 vec, float dist, float radius)
{
	if (dist < EPSILON)
		return float2(0.0, 1.0);
	const float C = -12.0 / (PI * pow(radius, 4.0));
	float value = max(0.0, radius - dist);
	return C * value * (vec / dist);
}

float spikyPow3Kernel(float dist, float radius)
{
	const float C = 10.0 / (PI * pow(radius, 5.0));
	float value = max(0.0, radius - dist);
	return C * pow(value, 3.0);
}

float2 spikyPow3Gradient(float2 vec, float dist, float radius)
{
	if (dist < EPSILON)
		return float2(0.0, 1.0);
	const float C = -30.0 / (PI * pow(radius, 4.0));
	float value = max(0.0, radius - dist);
	return C * pow(value, 2.0) * (vec / dist);
}

float smoothingPoly6Kernel(float dist, float radius)
{
	const float C = 4.0 / PI * pow(radius, 8.0);
	float value = max(0.0, radius * radius - dist * dist);
	return C * value * value * value;
}

#endif