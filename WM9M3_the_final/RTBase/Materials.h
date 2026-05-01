#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		

		
		float etaI = iorExt;//refractive index of incident medium
		float etaT = iorInt;//refractive index of transmitted medium
		
		if (cosTheta < 0.0f)
		{
			std::swap(etaI, etaT);
			cosTheta = -cosTheta;
		}
		
		float sinThetaI = 1.0f - cosTheta * cosTheta;
		// The sine of the angle of refraction
		float sinThetaT = (etaI / etaT) * (etaI / etaT) * sinThetaI;
		//total reflection
		if (sinThetaT >= 1.0f)
		{
			return 1.0f;
		}
		// The cosine of the refractive angle
		float cosThetaT = sqrtf(1.0f - sinThetaT);
		float eta = etaT / etaI;

		// Fresnel formul
		float rParallel = (cosTheta - eta * cosThetaT) /(cosTheta + eta * cosThetaT);
		// The reflection coefficient of the vertical polarization component
		float rPerpendicular = (eta * cosTheta - cosThetaT) /(eta * cosTheta + cosThetaT);
		// non-polarized light
		return 0.5f * ((rParallel * rParallel) + (rPerpendicular * rPerpendicular));
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		cosTheta = fabsf(cosTheta);

		float cos2 = cosTheta * cosTheta;
		float sin2 = 1.0f - cos2;

		Colour eta2 = ior * ior;
		Colour k2 = k * k;

		Colour cos2Colour(cos2, cos2, cos2);
		Colour sin2Colour(sin2, sin2, sin2);
		// 2*n*cos(theta)
		Colour twoEtaCos = ior * (2.0f * cosTheta);
		// n*n+ k*k
		Colour eta2PlusK2 = eta2 + k2;
		//Parallel component
		Colour rParallel =((eta2PlusK2 * cos2) - twoEtaCos + sin2Colour) /((eta2PlusK2 * cos2) + twoEtaCos + sin2Colour);
		//vertical component
		Colour rPerpendicular =(eta2PlusK2 - twoEtaCos + cos2Colour) /(eta2PlusK2 + twoEtaCos + cos2Colour);

		return (rParallel + rPerpendicular) * 0.5f;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		float cosTheta = fabsf(wi.z);
		
		float cos2 = cosTheta * cosTheta;
		float tan2 = (1.0f - cos2) / cos2;
		//Lambda fomula for GGX
		return 0.5f * (sqrtf(1.0f + alpha * alpha * tan2) - 1.0f);
	}

	
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		//Smith method for GGX
		// G(wi, wo) = G1(wi) * G1(wo)= 1 / ((1 + Lambda(wi)) * (1 + Lambda(wo)))
		return 1.0f / ((1.0f + lambdaGGX(wi, alpha)) * (1 + lambdaGGX(wo, alpha)));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		if (h.z <= 0.0f)
		{
			return 0.0f;
		}
		float alpha2 = alpha * alpha;
		float cos2 = h.z * h.z;
		float denom = cos2 * (alpha2 - 1.0f) + 1.0f;
		//ggx fomula
		return alpha2 / (M_PI * denom * denom);
	}
};

class BSDF
{
public:
	Colour emission;
	//for denoiser
	virtual Colour getAlbedo(const ShadingData& shadingData) = 0;

	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wiLocal);
		//p(x)/pi
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return shadingData.frame.toWorld(wiLocal);
	}
	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		//p(x)/pi
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	Colour getAlbedo(const ShadingData& shadingData){
		//for denoiser
		return albedo->sample(shadingData.tu, shadingData.tv);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		//ω_r = (-ωx, -ωy, ωz)
		Vec3 wiLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);

		

		pdf = 1.0f;
		// contribution = f_r* cos(thetai) / pdf
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wiLocal.z;

		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return albedo->sample(shadingData.tu, shadingData.tv);

	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float r1 = sampler->next();
		float r2 = sampler->next();
		float phi = 2.0f * M_PI * r1;
		//cos(thetam) = sqrt((1 - u) / (1 + (alpha^2 - 1)u))
		float cosTheta = sqrtf((1.0f - r2) / (1.0f + (alpha * alpha - 1.0f) * r2));
		float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
		Vec3 hLocal(cosf(phi) * sinTheta, sinf(phi) * sinTheta, cosTheta);

		if (Dot(woLocal, hLocal) < 0.0f)
		{
			hLocal = -hLocal;
		}
		//wi = 2 (wo dot h) h - wo
		Vec3 wiLocal = (hLocal * (2.0f * Dot(woLocal, hLocal))) - woLocal;
		Vec3 wi = shadingData.frame.toWorld(wiLocal);
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0.0f || woLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		//middle vector
		Vec3 h = (wiLocal + woLocal).normalize();
		
		//F(wo)
		Colour F = ShadingHelper::fresnelConductor(fabs(Dot(wiLocal, h)), eta, k);
		//D(wm)
		float D = ShadingHelper::Dggx(h, alpha);
		// G(wi, wo)
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		//f_r = F * D * G / (4 * cos(thetai) * cos(thetao))
		return albedo->sample(shadingData.tu, shadingData.tv) * F * (D * G / (4.0f * wiLocal.z * woLocal.z));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0.0f || woLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		Vec3 h = (wiLocal + woLocal).normalize();
		// p(ωi) = D(ωm) * cos(θm) / (4 * (ωo · ωm))
		return ShadingHelper::Dggx(h, alpha) * h.z / (4.0f * fabsf(Dot(woLocal, h)));
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return albedo->sample(shadingData.tu, shadingData.tv);

	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		
		float cosThetaO = woLocal.z;
		// Determine whether to enter or exit
		bool entering = cosThetaO > 0.0f;


		float etaI;
		float etaT;
		if (entering) {
			etaI = extIOR;
			etaT = intIOR;
		}
		else {
			
			etaI = intIOR;
			etaT = extIOR;
		}
		float eta = etaI / etaT;

		float absCosThetaO = fabs(cosThetaO);

		
		float F = ShadingHelper::fresnelDielectric(cosThetaO, intIOR, extIOR);

		// snell fomula etai sin(thetai) = etat sin(thett)
		
		float sin2ThetaO = 1.0f - absCosThetaO * absCosThetaO;
		// sin^2(thetat) = (etai / etat)^2 * sin^2(thetao)
		float sin2ThetaI = eta * eta * sin2ThetaO;

		
		bool totalInternalReflection = sin2ThetaI >= 1.0f;

		Vec3 wiLocal;

		
		if (totalInternalReflection || sampler->next() < F)
		{
			
			wiLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);

		
			//reflection or total internal reflection
			if (totalInternalReflection) {
				pdf = 1.0f;
			}else {
				pdf = F;
			}

		
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (pdf / fabs(wiLocal.z));
		}
		// refraction
		else
		{
			
			float cosThetaI = sqrtf(1.0f - sin2ThetaI);

			
			wiLocal = Vec3(-eta * woLocal.x, -eta * woLocal.y, entering ? -cosThetaI : cosThetaI);

			
			pdf = 1.0f - F;

			//ft = (1 - F) eta^2 / |cos(thetat)|
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * ((eta * eta) * pdf / fabs(wiLocal.z));
		}

		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return albedo->sample(shadingData.tu, shadingData.tv);

	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return albedo->sample(shadingData.tu, shadingData.tv);

	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return albedo->sample(shadingData.tu, shadingData.tv);

	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return albedo->sample(shadingData.tu, shadingData.tv);

	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	Colour getAlbedo(const ShadingData& shadingData) {
		return base->getAlbedo(shadingData);

	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};