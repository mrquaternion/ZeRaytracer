#include "object.hpp"

#include <cmath>
#include <cfloat>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include <iterator>


bool Object::intersect(Ray ray, Intersection &hit) const 
{
    // Assure une valeur correcte pour la coordonnée W de l'origine et de la direction
	// Vous pouvez commenter ces lignes si vous faites très attention à la façon de construire vos rayons.
    ray.origin[3] = 1;
    ray.direction[3] = 0;

    Ray local_ray(i_transform * ray.origin, i_transform * ray.direction);
	//!!! NOTE UTILE : pour calculer la profondeur dans localIntersect(), si l'intersection se passe à
	// ray.origin + ray.direction * t, alors t est la profondeur
	//!!! NOTE UTILE : ici, la direction peut être mise à l'échelle, alors vous devez la renormaliser
	// dans localIntersect(), ou vous aurez une profondeur dans le système de coordonnées local, qui
	// ne pourra pas être comparée aux intersections avec les autres objets.
    if (localIntersect(local_ray, hit)) 
	{
        // Assure la valeur correcte de W.
        hit.position[3] = 1;
        hit.normal[3] = 0;
        
		// Transforme les coordonnées de l'intersection dans le repère global.
        hit.position = transform * hit.position;
        hit.normal = (n_transform * hit.normal).normalized();
        
		return true;
    }
    return false;
}


bool Sphere::localIntersect(Ray const &ray, Intersection &hit) const 
{
	// Sphère avec rayon R : x^2 + y^2 + z^2 = R^2
	// H = P + (t * d) donc x = (P.x + (t * d.x))^2 et etc
	const double A = pow(ray.direction[0], 2.0) + pow(ray.direction[1], 2.0) + pow(ray.direction[2], 2.0);
	const double B = 2.0 * (ray.origin[0] * ray.direction[0] + ray.origin[1] * ray.direction[1] + ray.origin[2] * ray.direction[2]);
	const double C = pow(ray.origin[0], 2.0) + pow(ray.origin[1], 2.0) + pow(ray.origin[2], 2.0) - pow(this->radius, 2.0);
	const double delta = pow(B, 2.0) - 4.0 * A * C;

	if (delta < 0.0) return false;

	const double t1 = (-B - sqrt(delta)) / (2.0 * A);
	const double t2 = (-B + sqrt(delta)) / (2.0 * A);

	// On cherche la racine positive la plus proche
	double nearest = INFINITY;
	if (t1 > 0.0 && t1 < nearest) nearest = t1;
	if (t2 > 0.0 && t2 < nearest) nearest = t2;

	// pas nécessaire à cause de la prochaine condition, mais améliore les bugs potentiels
	if (nearest == INFINITY) return false;

	if (nearest < hit.depth)
	{
		const Vector H = ray.origin+ nearest * ray.direction;

		hit.position = H;
		hit.normal = H.normalized();
		hit.depth = nearest;

		// Si on est à l'intérieur de la sphère
		if (ray.direction.dot(hit.normal) > 0.0) hit.normal = -hit.normal;

		return true;
	}

	return false;
}


bool Plane::localIntersect(Ray const &ray, Intersection &hit) const
{
	// Pas de résolution de Ax + By + Cz + D = 0 car le repère local est en z = 0
	const double EPS = 1e-9;
	const auto normal = Vector(0, 0, 1);

	// dot(d, n) mais n = [0, 0, 1] donc résultat est juste la composante en 'z'
	if (abs(ray.direction[2]) < EPS) return false;

	// (Q - Q0) dot n = 0 -> (P + (t * d) - Q0) dot n = 0 -> t = (Q0 - P) / d = - P / d
	const double t = - ray.origin[2] / ray.direction[2];

	if (t <= 0.0) return false;

	if (t < hit.depth)
	{
		const Vector H = ray.origin + (t * ray.direction);

		hit.position = H;
		hit.normal = normal;
		hit.depth = t;

		return true;
	}

    return false;
}


bool Conic::localIntersect(Ray const &ray, Intersection &hit) const {
    // Tout comme un cylindre, il y a deux possibilités : les plans infinis du top/bottom et la surface du cône
	auto normal = Vector(0, 0, 1);

	// Plan infini top
	const auto Q0_top = Vector(0, 0, this->zMax);
	const double t_top = (Q0_top[2] - ray.origin[2]) / ray.direction[2];

	if (t_top > 0.0 && t_top < hit.depth) {
		const Vector H_top = ray.origin + ray.direction * t_top;
		if (pow(H_top[0], 2.0) + pow(H_top[1], 2.0) <= pow(this->radius2, 2.0))
		{
			hit.position = H_top;
			hit.normal = normal;
			hit.depth = t_top;

			return true;
		}
	}

	// Plan infini bottom
	const auto Q0_bot = Vector(0, 0, this->zMin);
	const double t_bot = (Q0_bot[2] - ray.origin[2]) / ray.direction[2];

	if (t_bot > 0.0 && t_bot < hit.depth) {
		const Vector H_bot = ray.origin + ray.direction * t_bot;
		if (pow(H_bot[0], 2.0) + pow(H_bot[1], 2.0) <= pow(this->radius1, 2.0))
		{
			hit.position = H_bot;
			hit.normal = normal;
			hit.depth = t_bot;

			return true;
		}
	}

	// Surface du cône : x^2 + y^2 = z^2
	double k = (this->radius2 - this->radius1) / (this->zMax - this->zMin);
	const double dx = ray.direction[0];
	const double dy = ray.direction[1];
	const double dz = ray.direction[2];
	const double ox = ray.origin[0];
	const double oy = ray.origin[1];
	const double oz = ray.origin[2];

	const double A = (dx * dx) + (dy * dy) - (k * k * dz * dz);
	double B = 2.0 * ((ox * dx) + (oy * dy) - (k * k) * (oz - zMin) * dz - (k * radius1 * dz));
	double C = (ox * ox) + (oy * oy) - (radius1 + k * (oz - zMin)) * (radius1 + k * (oz - zMin));
	const double delta = (B * B) - 4.0 * A * C;

	// Même truc qu'avec la sphère
	if (delta < 0.0) return false;

	const double t1 = (-B - sqrt(delta)) / (2.0 * A);
	const double t2 = (-B + sqrt(delta)) / (2.0 * A);

	double nearest = INFINITY;
	if (t1 > 0.0 && t1 < nearest) nearest = t1;
	if (t2 > 0.0 && t2 < nearest) nearest = t2;

	if (nearest == INFINITY) return false;

	const Vector H = ray.origin + nearest * ray.direction;
	if ((nearest < hit.depth) && (H[2] >= this->zMin) && (H[2] <= this->zMax))
	{
		hit.position = H;
		hit.normal = Vector(2.0 * H[0], 2.0 * H[1], 2.0 * k * (this->radius1 + k * (H[2] - zMin))); // normal du cône : [2x, 2y, 2z]
		hit.depth = nearest;

		return true;
	}

    return false;
}


// Intersections !
bool Mesh::localIntersect(Ray const &ray, Intersection &hit) const
{
	// Test de la boite englobante
	double tNear = -DBL_MAX, tFar = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (ray.direction[i] == 0.0) {
			if (ray.origin[i] < bboxMin[i] || ray.origin[i] > bboxMax[i]) {
				// Rayon parallèle à un plan de la boite englobante et en dehors de la boite
				return false;
			}
			// Rayon parallèle à un plan de la boite et dans la boite : on continue
		}
		else {
			double t1 = (bboxMin[i] - ray.origin[i]) / ray.direction[i];
			double t2 = (bboxMax[i] - ray.origin[i]) / ray.direction[i];
			if (t1 > t2) std::swap(t1, t2); // Assure t1 <= t2

			if (t1 > tNear) tNear = t1; // On veut le plus lointain tNear.
			if (t2 < tFar) tFar = t2; // On veut le plus proche tFar.

			if (tNear > tFar) return false; // Le rayon rate la boite englobante.
			if (tFar < 0) return false; // La boite englobante est derrière le rayon.
		}
	}
	// Si on arrive jusqu'ici, c'est que le rayon a intersecté la boite englobante.

	// Le rayon intersecte la boite englobante, donc on teste chaque triangle.
	bool isHit = false;
	for (size_t tri_i = 0; tri_i < triangles.size(); tri_i++) {
		Triangle const &tri = triangles[tri_i];

		if (intersectTriangle(ray, tri, hit)) {
			isHit = true;
		}
	}
	return isHit;
}

double Mesh::implicitLineEquation(double p_x, double p_y,
	double e1_x, double e1_y,
	double e2_x, double e2_y) const
{
	return (e2_y - e1_y)*(p_x - e1_x) - (e2_x - e1_x)*(p_y - e1_y);
}

bool Mesh::intersectTriangle(Ray const &ray,
	Triangle const &tri,
	Intersection &hit) const
{
	// Extrait chaque position de sommet des données du maillage.
	Vector const &p0 = positions[tri[0].pi];
	Vector const &p1 = positions[tri[1].pi];
	Vector const &p2 = positions[tri[2].pi];

	//!!! NOTE UTILE : pour le point d'intersection, sa normale doit satisfaire hit.normal.dot(ray.direction) < 0
	// Sauf qu'ici, on doit résoudre l'équation du plan Ax + By + Cz + D = 0
	// D = - (Ax + By + Cz)
	const Vector p0p1 = p1 - p0;
	const Vector p0p2 = p2 - p0;
	const Vector normal = p0p1.cross(p0p2).normalized();

	const double denom = normal.dot(ray.direction);

	if (abs(denom) < 1e-9) return false;

	// A * (P.x + (t * dir.x)) + B * (P.y + (t * dir.y)) + C * (P.z + (t * dir.z)) + D = 0
	const double D = - normal.dot(p0);
	const double t = - (normal.dot(ray.origin) + D) / denom;
	const Vector H = ray.origin + t * ray.direction;

	if (t <= 0.0) return false;

	if (t < hit.depth)
	{
		int idx1, idx2;
		const double absX = abs(normal[0]);
		const double absY = abs(normal[1]);
		const double absZ = abs(normal[2]);
		if (absZ >= absX && absZ >= absY)
		{ // plan [x, y]
			idx1 = 0;
			idx2 = 1;
		}
		else if (absY >= absX && absY >= absZ)
		{ // plan [x, z]
			idx1 = 0;
			idx2 = 2;
		}
		else
		{ // plan [y, z]
			idx1 = 1;
			idx2 = 2;
		}

		const double sign1 = this->implicitLineEquation(H[idx1], H[idx2], p0[idx1], p0[idx2], p1[idx1], p1[idx2]);
		const double sign2 = this->implicitLineEquation(H[idx1], H[idx2], p1[idx1], p1[idx2], p2[idx1], p2[idx2]);
		const double sign3 = this->implicitLineEquation(H[idx1], H[idx2], p2[idx1], p2[idx2], p0[idx1], p0[idx2]);

		if (((sign1 < 0.0) && (sign2 < 0.0) && (sign3 < 0.0)) ||
			((sign1 > 0.0) && (sign2 > 0.0) && (sign3 > 0.0)))
		{
			hit.position = H;
			hit.normal = normal;
			hit.depth = t;

			// Si on est à l'intérieur du mesh
			if (ray.direction.dot(hit.normal) > 0.0) hit.normal = -hit.normal;

			return true;
		}
	}

	return false;
}
