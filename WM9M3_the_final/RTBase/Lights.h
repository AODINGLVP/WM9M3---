#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		

		Vec3 localDir = SamplingDistributions::uniformSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformHemispherePDF(localDir);

		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(localDir);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<float> cdf;
	float totalweight = 0;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		cdf.resize(env->width * env->height);
		for (int i = 0; i < env->height; i++)
		{//weigth=luminance * sin(theta)
			float theta = sinf(((float)i / (float)env->height) * M_PI);
			for (int j = 0; j < env->width; j++)
			{
				totalweight += (env->texels[(i * env->width) + j].Lum() * theta);
				cdf[(i * env->width) + j] = totalweight;
			}
		}
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		if (totalweight <= 0.0f) {
			// If the environment map has no luminance, fall back to uniform sampling
			Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::uniformSpherePDF(wi);
			reflectedColour = evaluate(wi);
			return wi;
		}
		// Sample a direction based on the luminance of the environment map
		float randomnumber = sampler->next() * totalweight;
		
		int index = (int)(std::lower_bound(cdf.begin(), cdf.end(), randomnumber) - cdf.begin());

		float u = index % env->width / (float)env->width;
		float v = index / env->width / (float)env->height;

		float phi = u * 2.0f * M_PI;
		float theta = v * M_PI;


		float sinTheta = sinf(theta);
		// Convert spherical coordinates to Cartesian coordinates
		Vec3 wi(cosf(phi) * sinTheta,cosf(theta),sinf(phi) * sinTheta);
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(wi);
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		if (totalweight <= 0.0f)
		{
			return SamplingDistributions::uniformSpherePDF(wi);
		}

		float u = atan2f(wi.z, wi.x);
		if(u < 0.0f)
			u += 2.0f * M_PI; 

		u = u / (2.0f * M_PI);

		float v = acosf( wi.y) / M_PI;
		//uv to pixel coordinates
		int x = std::min((int)(u * env->width), env->width - 1);
		int y = std::min((int)(v * env->height), env->height - 1);

		float theta = (((float)y + 0.5f) / (float)env->height) * M_PI;
		float sinTheta = sinf(theta);
		//Current pixel brightness
		float lum = env->texels[(y * env->width) + x].Lum();
		//PDF of sampling this pixel
		float pixelPDF = (lum * sinTheta) / totalweight;

		float solidAngle =(2.0f * M_PI / (float)env->width) *(M_PI / (float)env->height) *sinTheta;
		return pixelPDF / solidAngle;

	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		ShadingData shadingData;
		Colour reflectedColour;

		return sample(shadingData, sampler, reflectedColour, pdf);
	}
};