#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

#include <atomic>
#include<mutex>
struct shadingarea {
	int xstart;
	int ystart;
	int xend;
	int yend;
};
struct VirtualPointLight
{
	ShadingData shadingData;
	Colour throughput;
};

std::vector<VirtualPointLight> vpls;
int instantRadiosityPathCount = 128;
int instantRadiosityMaxDepth = 3;
float instantRadiosityScale = 1.0f;
int lightTracingPathCount = 64800;
class RayTracer
{
public:
	int mode = 0;
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int x_block;
	int y_block;
	int numProcs;
	std::vector<shadingarea> shadingareas;
	std::vector<std::jthread>Jthreads;

	std::atomic<int>workingprocs;
	std::atomic<bool> stopRendering;
	
	
	std::atomic<int> currentarea;
	std::mutex mtx;
	int jthreadnum;
	void chagemode(int _mode) {
		mode = _mode;
		film->clear();
	}
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		currentarea = 0;
		stopRendering = true;
	
	
		
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		workingprocs = 0;
		jthreadnum = std::min(8, numProcs);
		for (int i = 0; i < jthreadnum; i++)
		{
			Jthreads.emplace_back(&RayTracer::mutilrender, this, i);
		}
		x_block = ((int)scene->camera.width + 19) / 20;
		y_block = ((int)scene->camera.height + 19) / 20;
		int currentx = 0;
		int currenty = 0;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < 20; j++) {
				shadingarea scv;
				scv.xstart = currentx;
				scv.ystart = currenty;
				scv.xend = std::min(currentx + x_block, (int)scene->camera.width);
				scv.yend = std::min(currenty + y_block, (int)scene->camera.height);
				shadingareas.push_back(scv);
				currentx += x_block;
				if (currentx >= (int)scene->camera.width)break;
			}
			currentx = 0;
			currenty += y_block;
			if (currenty >= (int)scene->camera.height)break;
		}
		clear();
	}

	void clear()
	{
		film->clear();
	}
	void buildVPLs(Sampler* sampler)
	{
		vpls.clear();
		
		for (int i = 0; i < instantRadiosityPathCount; i++)
		{
			float lightPMF = 0.0f;

			
			Light* light = scene->sampleLight(sampler, lightPMF);

			if (!light || lightPMF <= 0.0f)
				continue;

			

			ShadingData fakeShading;
			fakeShading.x = Vec3(0, 0, 0);

			Colour Le;
			float lightPDF = 0.0f;
			//Sample a point from the light source
			Vec3 lightPos = light->sample(fakeShading, sampler, Le, lightPDF);

			if (lightPDF <= 0.0f || Le.Lum() <= 0.0f)
				continue;
			// Calculate the surface normal of the light source
			Vec3 nLight = light->normal(fakeShading, Vec3(0, 1, 0));
			nLight = nLight.normalize();
			// Sample a direction in the local coordinate system
			Vec3 localDir = SamplingDistributions::cosineSampleHemisphere(sampler->next(),sampler->next());

			Frame frame(nLight);
			Vec3 dir = frame.toWorld(localDir).normalize();
			//pdf=cos(theta)/pi
			float pdfDir = localDir.z / M_PI;

			if (pdfDir <= 0.0f)
				continue;

			float cosLight = std::max(0.0f, Dot(nLight, dir));

			if (cosLight <= 0.0f)
				continue;

			// Initial energy of the light source path
			Colour throughput = Le * (cosLight / (lightPDF * lightPMF * pdfDir));

			Ray ray;
			ray.init(lightPos + dir * EPSILON, dir);
			// Track the path and generate virtual point light sources.
			for (int depth = 0; depth < instantRadiosityMaxDepth; depth++)
			{
				IntersectionData intersection = scene->traverse(ray);
				ShadingData shadingData = scene->calculateShadingData(intersection, ray);

				if (shadingData.t == FLT_MAX)
					break;

				if (shadingData.bsdf->isLight())
					break;
				//generate vpl
				if (!shadingData.bsdf->isPureSpecular())
				{
					VirtualPointLight vpl;
					vpl.shadingData = shadingData;
					vpl.throughput = throughput;
					vpls.push_back(vpl);
				}

				Colour reflectedColour;
				float pdf = 0.0f;

				Vec3 wi = shadingData.bsdf->sample(shadingData,sampler,reflectedColour,pdf);

				if (pdf <= 0.0f)
					break;

				float cosTheta;

				if (shadingData.bsdf->isPureSpecular())
				{
					cosTheta = fabs(Dot(shadingData.sNormal, wi));
				}
				else
				{
					cosTheta = std::max(0.0f, Dot(shadingData.sNormal, wi));
				}

				if (cosTheta <= 0.0f)
					break;
				// v
				throughput = throughput * reflectedColour * (cosTheta / pdf);

				if (throughput.Lum() <= 0.0f)
					break;

				ray.init(shadingData.x + wi * EPSILON, wi);
			}
		}
	}
	Colour instantRadiosity(Ray& r, Sampler* sampler)
	{
		Colour pathThroughput(1.0f, 1.0f, 1.0f);
		Ray ray = r;
		//for mirror,max 10 bounce
		for (int bounce = 0; bounce < 10; bounce++)
		{
			IntersectionData intersection = scene->traverse(ray);
			ShadingData shadingData = scene->calculateShadingData(intersection, ray);
			//hit nothing
			if (shadingData.t == FLT_MAX)
			{
				return pathThroughput * scene->background->evaluate(ray.dir);
			}
			//hit light
			if (shadingData.bsdf->isLight())
			{
				return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			//mirror reflection,trace to the next bounce
			if (shadingData.bsdf->isPureSpecular())
			{
				Colour reflectedColour;
				float pdf = 0.0f;
				// Sample the BSDF to get the reflected direction
				Vec3 wi = shadingData.bsdf->sample(shadingData,sampler,reflectedColour,pdf);

				if (pdf <= 0.0f)
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}

				float cosTheta = fabs(Dot(shadingData.sNormal, wi));

				if (cosTheta <= 0.0f)
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
				// Update the path throughput
				pathThroughput = pathThroughput * reflectedColour * (cosTheta / pdf);

				ray.init(shadingData.x + wi * EPSILON, wi);
				continue;
			}
			//direction light
			Colour result = pathThroughput * computeDirect(shadingData, sampler);


			Colour indirect(0.0f, 0.0f, 0.0f);

			// Iterate over all virtual point lights
			for (int i = 0; i < (int)vpls.size(); i++)
			{
				VirtualPointLight& vpl = vpls[i];
				//get direction
				Vec3 d = vpl.shadingData.x - shadingData.x;
				float dist2 = Dot(d, d);
				dist2 = std::max(dist2, 0.01f);

				float dist = sqrtf(dist2);
				Vec3 wi = d / dist;

				float cosSurface = std::max(0.0f, Dot(shadingData.sNormal, wi));
				float cosVPL = std::max(0.0f, Dot(vpl.shadingData.sNormal, -wi));

				if (cosSurface <= 0.0f || cosVPL <= 0.0f)
					continue;
				//visibility test
				if (!scene->visible(shadingData.x + wi * EPSILON,vpl.shadingData.x - wi * EPSILON))
					continue;
				//bsdf of camera and vpl
				Colour fCamera = shadingData.bsdf->evaluate(shadingData, wi);
				Colour fVPL = vpl.shadingData.bsdf->evaluate(vpl.shadingData, -wi);

				//G = (cossurface * cosvpl) / r^2
				float G = (cosSurface * cosVPL) / dist2;
				// Monte Carlo estimation
				indirect = indirect + fCamera * fVPL * vpl.throughput * G;
			}

			indirect = indirect /  instantRadiosityPathCount;

			return result + pathThroughput * indirect * instantRadiosityScale;
		}

		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		
		if (shadingData.bsdf->isPureSpecular())
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float lightPMF = 0.0f;
		Light* light = scene->sampleLight(sampler, lightPMF);

	
		if (light == NULL || lightPMF <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		
		Colour Le;       
		float lightPDF = 0.0f; 
		// Sample a point or a direction from the selected light source
		Vec3 lightSample = light->sample(shadingData, sampler, Le, lightPDF);

		if (lightPDF <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Vec3 wi;       
		float dist2 = 0.0f;
		float dist = 0.0f;

	
		if (light->isArea())
		{
			// From the shading point to the sampling point of the light source
			Vec3 d = lightSample - shadingData.x;
			dist2 = Dot(d, d);
			dist = sqrtf(dist2);

			
			if (dist <= EPSILON)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}

			wi = d / dist;

			// visibility test
			if (!scene->visible(shadingData.x, lightSample))
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}

			
			Vec3 nLight = light->normal(shadingData, wi);

			
			float cosSurface = std::max(0.0f, Dot(shadingData.sNormal, wi));

			
			float cosLight = std::max(0.0f, Dot(nLight, -wi));

			
			if (cosSurface <= 0.0f || cosLight <= 0.0f)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}

			// BSDF evaluation
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);

			
			// G = (cosθsurface * cosθlight) / r^2
			float G = (cosSurface * cosLight) / dist2;

			//Monte Carlo estimation
			return (f * Le) * (G / (lightPDF * lightPMF));
		}

		//other light
		else
		{
			// get direction
			wi = lightSample.normalize();

			float cosSurface = std::max(0.0f, Dot(shadingData.sNormal, wi));

			if (cosSurface <= 0.0f)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}

			// BSDF
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			float bsdfPDF = shadingData.bsdf->PDF(shadingData, wi);
			// visibility test
			if (!scene->visible(shadingData.x, shadingData.x + wi * 1000000.0f))
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			float lightPDFAll = lightPDF * lightPMF;
			float misWeight = (lightPDFAll* lightPDFAll) / (lightPDFAll* lightPDFAll + bsdfPDF*bsdfPDF);

			//Monte Carlo estimation
			return (f * Le) * (cosSurface* misWeight / (lightPDF * lightPMF));
		}
	}


	void drawFilmToCanvas()
	{
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r = 0;
				unsigned char g = 0;
				unsigned char b = 0;
				film->tonemap((int)x, (int)y, r, g, b);
				canvas->draw((int)x, (int)y, r, g, b);
			}
		}
	}

	void connectLightPathToCamera(const ShadingData& shadingData, const Colour& pathThroughput)
	{
		
		if (shadingData.bsdf->isPureSpecular())
		{
			return;
		}

		float px = 0.0f;
		float py = 0.0f;
		
		// Project the current focus onto the camera.
		if (!scene->camera.projectOntoCamera(shadingData.x, px, py))
		{
			return;
		}

		// Calculate the direction and distance from the current intersection point to the camera.
		Vec3 toCamera = scene->camera.origin - shadingData.x;
		float distance = toCamera.length();
		if (distance <= EPSILON)
		{
			return;
		}

		Vec3 wi = toCamera / distance;
		//visiblity test
		if (!scene->visible(shadingData.x + wi * EPSILON, scene->camera.origin))
		{
			return;
		}

		//Located on the visible hemisphere
		float cosSurface = std::max(0.0f, Dot(shadingData.sNormal, wi));
		if (cosSurface <= 0.0f)
		{
			return;
		}

		
		Colour f = shadingData.bsdf->evaluate(shadingData, wi);
		if (f.Lum() <= 0.0f)
		{
			return;
		}

		
	
		film->splat(px, py, pathThroughput * f * cosSurface);
	}

	void lightTracePass(Sampler* sampler)
	{
		
		const int maxDepth = 10;
		for (int i = 0; i < lightTracingPathCount; i++)
		{
			float lightPMF = 0.0f;
			//sample a light
			Light* light = scene->sampleLight(sampler, lightPMF);
			if (!light || lightPMF <= 0.0f)
			{
				continue;
			}

			float positionPDF = 0.0f;
			float directionPDF = 0.0f;
			// sample a position and a direction from the light source
			Vec3 lightPos = light->samplePositionFromLight(sampler, positionPDF);
			Vec3 dir = light->sampleDirectionFromLight(sampler, directionPDF).normalize();

			
			if (positionPDF <= 0.0f || directionPDF <= 0.0f)
			{
				continue;
			}

			ShadingData lightShading;
			lightShading.x = lightPos;
			
			Colour Le = light->evaluate(-dir);
			
			if (Le.Lum() <= 0.0f)
			{
				continue;
			}

			float lightCosTerm = 1.0f;
			if (light->isArea())
			{
				
				Vec3 nLight = light->normal(lightShading, dir).normalize();
				lightCosTerm = std::max(0.0f, Dot(nLight, dir));
				if (lightCosTerm <= 0.0f)
				{
					continue;
				}
			}

			// Initial energy of the light path
			Colour pathThroughput = Le * (lightCosTerm / (lightPMF * positionPDF * directionPDF));

			Ray ray;
			ray.init(lightPos + dir * EPSILON, dir);

			
			for (int depth = 0; depth < maxDepth; depth++)
			{
				IntersectionData intersection = scene->traverse(ray);
				ShadingData shadingData = scene->calculateShadingData(intersection, ray);

				// The light ceases to follow this path once it leaves the scene.
				if (shadingData.t == FLT_MAX)
				{
					break;
				}

				// Avoid repeatedly hitting the luminous object
				if (shadingData.bsdf->isLight())
				{
					break;
				}

				// Attempt to project onto the camera
				connectLightPathToCamera(shadingData, pathThroughput);

				Colour reflectedColour;
				float pdf = 0.0f;
				
				Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, reflectedColour, pdf);
				if (pdf <= 0.0f)
				{
					break;
				}

				float cosTheta;
				if (shadingData.bsdf->isPureSpecular())
				{
					
					cosTheta = fabs(Dot(shadingData.sNormal, wi));
				}
				else
				{
					
					cosTheta = std::max(0.0f, Dot(shadingData.sNormal, wi));
				}

				if (cosTheta <= 0.0f)
				{
					break;
				}

				// update path throughput
				pathThroughput = pathThroughput * reflectedColour * (cosTheta / pdf);
				if (pathThroughput.Lum() <= 0.0f)
				{
					break;
				}

				
				ray.init(shadingData.x + wi * EPSILON, wi);
			}
		}
	}
	
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool previousSpecular = true)
	{
		
		const int maxDepth = 64;

		//end recursion
		if (depth > maxDepth)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		
		IntersectionData intersection = scene->traverse(r);

	
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

	
		if (shadingData.t == FLT_MAX)
		{
			
			if (depth == 0 || previousSpecular)
			{
				// The first ray either bounces off the mirror surface or returns the ambient light.
				return pathThroughput * scene->background->evaluate(r.dir);
			}

			
			return Colour(0.0f, 0.0f, 0.0f);
		}

	
		if (shadingData.bsdf->isLight())
		{
			
			if (depth == 0 || previousSpecular)
			{
				// The first ray either bounces off the mirror surface or returns the ambient light.
				return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
			}

			
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Direct illumination
		Colour radiance = pathThroughput * computeDirect(shadingData, sampler);

		
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());

		Colour reflectedColour;
		float pdf = 0.0f;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, reflectedColour, pdf);

		//Sampling point is invalid
		if (pdf <= 0.0f)
		{
			return radiance;
		}

		
		float cosTheta;

		if (shadingData.bsdf->isPureSpecular())
		{
			//the absolute value should be taken because reflection or refraction may occur.
			cosTheta = fabs(Dot(shadingData.sNormal, wi));
		}
		else
		{
			cosTheta = std::max(0.0f, Dot(shadingData.sNormal, wi));
		}

		//direction points to the back side of the surface ,invalid.
		if (cosTheta <= 0.0f)
		{
			return radiance;
		}

		// BSDF evaluation
		Colour f = shadingData.bsdf->evaluate(shadingData, wi);
		// New path throughput
		Colour nextPathThroughput = pathThroughput * reflectedColour * (cosTheta / pdf);

		if (nextPathThroughput.Lum() <= 0.0f)
		{
			return radiance;
		}

		//Russian Roulette
		if (depth >= 3)
		{
			
			
			//If the random number is greater than the continuation probability, then terminate this path.
			if (sampler->next() > nextPathThroughput.Lum())
			{
				
				return radiance;
			}

			//Compensation weight
			nextPathThroughput = nextPathThroughput / nextPathThroughput.Lum();
		}



		Ray nextRay;

	
		nextRay.init(shadingData.x + wi * EPSILON, wi);

		
		return radiance + pathTrace(nextRay, nextPathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	//for OIDN denoiser
	void GetOriginal(Ray& r, Colour& albedoAOV, Vec3& normalAOV)
	{
		
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			if (shadingData.bsdf->isLight())
			{
				albedoAOV = Colour(1.0f, 1.0f, 1.0f);
			}
			else
			{
				//get orginal albedo 
				albedoAOV = shadingData.bsdf->getAlbedo(shadingData);
					
			}

			Vec3 n = shadingData.sNormal;
			
				n = n.normalize();
			
			normalAOV = n;
			return;
		}

		albedoAOV = Colour(0.0f, 0.0f, 0.0f);
		normalAOV = Vec3(0.0f, 0.0f, 0.0f);
	}
	
	void startRenderThreads()
	{
		if (mode == 2)
		{
			lightTracePass(&samplers[0]);
			film->incrementSPP();
			drawFilmToCanvas();
			return;
		}
		
		{
			if (mode == 1)
			{
				buildVPLs(&samplers[0]);
			}
		}
		film->incrementSPP();
		currentarea = 0;
		stopRendering = false;
		workingprocs = jthreadnum;
		stopRendering.notify_all();
		while (true)
		{
			int remaining = workingprocs;
			if (remaining == 0)
			{
				break;
			}
			workingprocs.wait(remaining);
		}
	}
	void mutilrender(int number) {
		while (1) {
			stopRendering.wait(true);
		

			while (true)
			{
				int nowareapoint;
				{
					mtx.lock();
					nowareapoint = currentarea;
					currentarea++;
					mtx.unlock();
				}

				if (nowareapoint >= (int)shadingareas.size()) {
					workingprocs--;
					if(workingprocs==0)
					workingprocs.notify_all();
					stopRendering = true;
					break;
				}


				int minx;
				int miny;
				int maxx;
				int maxy;

				minx = shadingareas[nowareapoint].xstart;
				miny = shadingareas[nowareapoint].ystart;
				maxx = shadingareas[nowareapoint].xend;
				maxy = shadingareas[nowareapoint].yend;



				for (int y = miny; y < maxy; y++)
				{
					for (int x = minx; x < maxx; x++)
					{
						float px = x + 0.5f;
						float py = y + 0.5f;
						Ray ray = scene->camera.generateRay(px, py);
						Colour pathThroughput(1.0f, 1.0f, 1.0f);
						//Colour col = viewNormals(ray);
						//Colour col = albedo(ray);
						//Colour col = direct(ray, &samplers[0]);
						Colour col;
						if (mode == 0) {
							 col = pathTrace(ray, pathThroughput, 0, &samplers[number]);
						}
						else if (mode == 1) {
							 col = instantRadiosity(ray, &samplers[number]);
						}
						else {
							 col = direct(ray, &samplers[number]);
						}
						
						
						Colour albedoAOV;
						Vec3 normalAOV;
						
						GetOriginal(ray, albedoAOV, normalAOV);
						film->splat(px, py, col);
						film->splatOIDN(px, py, albedoAOV, normalAOV);
						unsigned char r = (unsigned char)(col.r * 255);
						unsigned char g = (unsigned char)(col.g * 255);
						unsigned char b = (unsigned char)(col.b * 255);
						film->tonemap(x, y, r, g, b);
						canvas->draw(x, y, r, g, b);
					}
				}
			}
		}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
	bool saveDenoisedPNG(std::string filename)
	{
		
		return film->saveDenoisedPNG(filename);
	}
};
