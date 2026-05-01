#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};



#define EPSILON 0.001f

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		t = std::min(tentry, texit);
		return texit > 0 && tentry <= texit;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 s = (min - r.o) * r.invDir;
		Vec3 l = (max - r.o) * r.invDir;
		Vec3 s1 = Min(s, l);
		Vec3 l1 = Max(s, l);
		float ts = std::max(s1.x, std::max(s1.y, s1.z));
		float tl = std::min(l1.x, std::min(l1.y, l1.z));
		return tl >= 0 && ts <= tl;
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = n.dot(r.dir);
		if (denom == 0) { return false; }
		t = (d - n.dot(r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || v > 1) { return false; }
		if (u + v > 1) { return false; }
		return true;
	}
	
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		float sqrt_r1 = sqrt(r1);
		float a = 1 - sqrt_r1;
		float b = sqrt_r1 * (1 - r2);
		float c = sqrt_r1 * r2;

		Vec3 p =
			vertices[0].p * a +
			vertices[1].p * b +
			vertices[2].p * c;
		pdf = 1.0f / area;
		return p;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
	AABB getBounds() 
	{
		AABB box;
		box.extend(vertices[0].p);
		box.extend(vertices[1].p);
		box.extend(vertices[2].p);
		return box;
	}
};



struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	std::vector<unsigned int> triangleIndices;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	bool isLeaf() const
	{
		return l == NULL && r == NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, const std::vector<unsigned int>& inputIndices)
	{
		
		bounds.reset();
		l = nullptr;
		r = nullptr;
		triangleIndices.clear();

		
		if (inputIndices.empty())
		{
			return;
		}

		//get parent bounds
		for (unsigned int i = 0; i < inputIndices.size(); i++)
		{
			const Triangle& tri = inputTriangles[inputIndices[i]];
			bounds.extend(tri.vertices[0].p);
			bounds.extend(tri.vertices[1].p);
			bounds.extend(tri.vertices[2].p);
		}

		
		if (inputIndices.size() <= MAXNODE_TRIANGLES)
		{
			triangleIndices = inputIndices;
			return;
		}

		float parent_area = bounds.area();
		float leaf_cost = (float)inputIndices.size() * TRIANGLE_COST;

		float best_cost = FLT_MAX;
		int best_axis = -1;
		int best_split = -1;
		std::vector<unsigned int> best_sorted_indices;

		
		for (int axis = 0; axis < 3; axis++)
		{
			std::vector<unsigned int> sorted_indices = inputIndices;

			
			std::sort(sorted_indices.begin(), sorted_indices.end(),
				[&](unsigned int a, unsigned int b)
				{
					Vec3 ca = inputTriangles[a].centre();
					Vec3 cb = inputTriangles[b].centre();

					if (axis == 0) return ca.x < cb.x;
					if (axis == 1) return ca.y < cb.y;
					return ca.z < cb.z;
				});
			// precompute bounding boxes for left and right subsets
			std::vector<AABB> LeftBounds(sorted_indices.size());
			std::vector<AABB> RightBounds(sorted_indices.size());
			AABB Leftbox;
			Leftbox.reset();
			for (int i = 0; i < (int)sorted_indices.size(); i++)
			{
				const Triangle& tri = inputTriangles[sorted_indices[i]];
				Leftbox.extend(tri.vertices[0].p);
				Leftbox.extend(tri.vertices[1].p);
				Leftbox.extend(tri.vertices[2].p);
				LeftBounds[i] = Leftbox;
			}
			AABB Rightbox;
			Rightbox.reset();
			for (int i = (int)sorted_indices.size() - 1; i >= 0; i--)
			{
				const Triangle& tri = inputTriangles[sorted_indices[i]];
				Rightbox.extend(tri.vertices[0].p);
				Rightbox.extend(tri.vertices[1].p);
				Rightbox.extend(tri.vertices[2].p);
				RightBounds[i] = Rightbox;
			}
			// evaluate SAH cost for each possible split
			for (int split = 1; split < (int)sorted_indices.size(); split++)
			{
				AABB left_bounds = LeftBounds[split - 1];
				AABB right_bounds = RightBounds[split];
				float left_area = left_bounds.area();
				float right_area = right_bounds.area();

				float cost =
					TRAVERSE_COST +
					(left_area / parent_area) * split * TRIANGLE_COST +
					(right_area / parent_area) * ((int)sorted_indices.size() - split) * TRIANGLE_COST;

				if (cost < best_cost)
				{
					best_cost = cost;
					best_axis = axis;
					best_split = split;
					best_sorted_indices = sorted_indices;
				}
			}
		}

		
		if (best_axis == -1 || leaf_cost <= best_cost)
		{
			triangleIndices = inputIndices;
			return;
		}

		
		std::vector<unsigned int> leftIndices(
			best_sorted_indices.begin(),
			best_sorted_indices.begin() + best_split
		);

		std::vector<unsigned int> rightIndices(
			best_sorted_indices.begin() + best_split,
			best_sorted_indices.end()
		);

	
		if (leftIndices.empty() || rightIndices.empty())
		{
			triangleIndices = inputIndices;
			return;
		}

	
		l = new BVHNode();
		r = new BVHNode();

		l->build(inputTriangles, leftIndices);
		r->build(inputTriangles, rightIndices);

		triangleIndices.clear();
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		float tBox;
		//check intersection with bounding box
		if (!bounds.rayAABB(ray, tBox))
		{
			return;
		}

		if (isLeaf())
		{// try all triangles in the leaf node
			for (unsigned int i = 0; i < triangleIndices.size(); i++)
			{
				unsigned int triID = triangleIndices[i];
				float t, u, v;
				if (triangles[triID].rayIntersect(ray, t, u, v))
				{
					if (t > EPSILON && t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = triID;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}

		if (l) l->traverse(ray, triangles, intersection);
		if (r) r->traverse(ray, triangles, intersection);
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		float tBox;
		if (!bounds.rayAABB(ray, tBox))
		{
			return true;
		}

		if (isLeaf())
		{
			for (unsigned int i = 0; i < triangleIndices.size(); i++)
			{
				unsigned int triID = triangleIndices[i];
				float t, u, v;
				if (triangles[triID].rayIntersect(ray, t, u, v))
				{// blocked
					if (t > EPSILON && t < maxT)
					{
						return false;
					}
				}
			}
			return true;
		}

		if (l && !l->traverseVisible(ray, triangles, maxT))
			return false;
		if (r && !r->traverseVisible(ray, triangles, maxT))
			return false;

		return true;
		
	}
};
