#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:


	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		
		float z = r1;

		
		//x^2 + y^2 + z^2 = 1
		float r = sqrtf(fmax(0.0f, 1.0f - z * z));

		//phi between 0 and 2PI
		float phi = 2.0f * M_PI * r2;

		//Polar to Cartesian coordinates conversion
		float x = r * cosf(phi);
		float y = r * sinf(phi);


		return Vec3(x, y, z);
	}

	static float uniformHemispherePDF(const Vec3 wi)
	{
		//only upwards hemisphere
		if (wi.z < 0.0f) return 0.0f;

		//pdf=1/area
		return 1.0f / (2.0f * M_PI);
	}



	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		//Cosine distribution
		float r = sqrtf(r1);

		//Uniform sampling angle
		float theta = 2.0f * M_PI * r2;

		//Polar to Cartesian coordinates conversion
		float x = r * cosf(theta);
		float y = r * sinf(theta);

		//Rise to the hemisphere
		float z = sqrtf(fmax(0.0f, 1.0f - r1));

		return Vec3(x, y, z);
	}

	static float cosineHemispherePDF(const Vec3 wi)
	{
		//only upwards hemisphere
		if (wi.z <= 0.0f) return 0.0f;

		//pdf=cos(theta)/pi
		return wi.z / M_PI;
	}



	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		
		float z = 1.0f - 2.0f * r1;

		//Circumferential section radius
		float r = sqrtf(fmax(0.0f, 1.0f - z * z));


		float phi = 2.0f * M_PI * r2;
		//Polar to Cartesian coordinates conversion
		float x = r * cosf(phi);
		float y = r * sinf(phi);


		return Vec3(x, y, z);
	}

	static float uniformSpherePDF(const Vec3& wi)
	{
		//pdf=1/area
		return 1.0f / (4.0f * M_PI);
	}
};