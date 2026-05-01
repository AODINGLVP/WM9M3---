#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"
#include <cmath>
#include <iostream>
#include <string>


#include <OpenImageDenoise/oidn.hpp>


// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class Film
{
public:

	Colour* film;
	Colour* albedoFilm;//The base color of the object's surface
	Vec3* normalFilm;// The normal of an object's surface
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;
	//for  albedo
	void splatNoise(Colour* target, const float x, const float y, const Colour& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25]; 
		unsigned int indices[25]; 
		unsigned int used = 0;
		float total = 0;//the total weight for normalisation
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					//calculate the filtering weight based on the distance between the pixel center and the sampling point (x, y)
					filterWeights[used] = filter->filter(px - x, py - y);
					// cumulative total weight
					total += filterWeights[used];
					used++;
				}
			}
		}
		//allocate the sampled color L according to the weights to each pixel
		for (int i = 0; i < used; i++) {
			target[indices[i]] = target[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	//for normal
	void splatNoise(Vec3* target, const float x, const float y, const Vec3& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25]; 
		unsigned int indices[25]; 
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		if (total <= 0.0f)
		{
			return;
		}
		for (int i = 0; i < used; i++) {
			target[indices[i]] = target[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	void splat(const float x, const float y, const Colour& L)
	{
		float filterWeights[25]; 
		unsigned int indices[25]; 
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		if (total <= 0.0f)
		{
			return;
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	void splatOIDN(const float x, const float y, const Colour& albedo, const Vec3& normal)
	{
		
		splatNoise(albedoFilm, x, y, albedo);
		splatNoise(normalFilm, x, y, normal);
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		// return a tonemapped pixel at coordinates x, y
		int idx = y * width + x;
		Colour c = film[idx];
		if (SPP > 0)
			c = c / (float)SPP;
		c = c * exposure;

		// reinhard global tone mapping
		c.r = c.r / (1.0f + c.r);
		c.g = c.g / (1.0f + c.g);
		c.b = c.b / (1.0f + c.b);
		// gamma correction
		c.r = powf(std::max(0.0f, c.r), 1.0f / 2.2f);
		c.g = powf(std::max(0.0f, c.g), 1.0f / 2.2f);
		c.b = powf(std::max(0.0f, c.b), 1.0f / 2.2f);
		r = (unsigned char)(std::min(1.0f, std::max(0.0f, c.r)) * 255);
		g = (unsigned char)(std::min(1.0f, std::max(0.0f, c.g)) * 255);
		b = (unsigned char)(std::min(1.0f, std::max(0.0f, c.b)) * 255);
	}



	void makeColourBuffer(Colour* input, std::vector<float>& output) const
	{
		// get average
		output.resize(width * height * 3);
		for (unsigned int i = 0; i < (width * height); i++)
		{
			Colour c = input[i] / (float)SPP;
			output[(i * 3) + 0] = c.r;
			output[(i * 3) + 1] = c.g;
			output[(i * 3) + 2] = c.b;
		}
	}

	void makeVec3Buffer(std::vector<float>& output) const
	{
		// get average
		output.resize(width * height * 3);
		for (unsigned int i = 0; i < (width * height); i++)
		{
			Vec3 n = normalFilm[i] / (float)SPP;

			
			output[(i * 3) + 0] = n.x;
			output[(i * 3) + 1] = n.y;
			output[(i * 3) + 2] = n.z;
		}
	}
	bool denoiseBuffer(const std::vector<float>& colour, const std::vector<float>& albedo, const std::vector<float>& normal, std::vector<float>& denoised) const
	{
		//OIDE
		denoised.resize(colour.size());
		oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();
		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", const_cast<float*>(colour.data()), oidn::Format::Float3, width, height);
		filter.setImage("albedo", const_cast<float*>(albedo.data()), oidn::Format::Float3, width, height);
		filter.setImage("normal", const_cast<float*>(normal.data()), oidn::Format::Float3, width, height);
		filter.setImage("output", denoised.data(), oidn::Format::Float3, width, height);
		filter.set("hdr", true);
		filter.set("cleanAux", true);
		filter.commit();
		filter.execute();
		return true;


	}
	static unsigned char tonemapHDRChannel(float value)
	{
		//Reinhard tone mapping
		value = value / (1.0f + value);
		//gamma correction
		value = powf(std::max(0.0f, value), 1.0f / 2.2f);
		return (unsigned char)(std::min(1.0f, std::max(0.0f, value)) * 255);
	}
	void writePNGFromHDRBuffer(std::string filename, const std::vector<float>& hdrpixels) const
	{
		// change OIDE hdr to ldr
		std::vector<unsigned char> ldrpixels(width * height * 3);
		for (unsigned int i = 0; i < (width * height); i++)
		{
			ldrpixels[(i * 3) + 0] = tonemapHDRChannel(hdrpixels[(i * 3) + 0]);
			ldrpixels[(i * 3) + 1] = tonemapHDRChannel(hdrpixels[(i * 3) + 1]);
			ldrpixels[(i * 3) + 2] = tonemapHDRChannel(hdrpixels[(i * 3) + 2]);
		}
		stbi_write_png(filename.c_str(), width, height, 3, ldrpixels.data(), width * 3);
	}
	bool saveDenoisedPNG(std::string filename)
	{
		
		std::vector<float> colour;
		std::vector<float> albedo;
		std::vector<float> normal;
		std::vector<float> denoised;//result from OIDN

		makeColourBuffer(film,colour);
		makeColourBuffer(albedoFilm,albedo);
		makeVec3Buffer(normal);

		bool denoisedOK = denoiseBuffer(colour, albedo, normal, denoised);
		writePNGFromHDRBuffer(filename, denoisedOK ? denoised : colour);
		return denoisedOK;
	}




	// Do not change any code below this line
	//I don't want to make any changes.
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		//allocate the albedo and normal buffers for OIDN
		albedoFilm = new Colour[width * height];
		normalFilm = new Vec3[width * height];
		clear();
		filter = _filter;
	}
	void clear()
	{
		
		memset(film, 0, width * height * sizeof(Colour));
		memset(albedoFilm, 0, width * height * sizeof(Colour));
		memset(normalFilm, 0, width * height * sizeof(Vec3));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}

	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}

};
