#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>

#include "raytracer.hpp"
#include "image.hpp"



void Raytracer::render(const char *filename, const char *depth_filename,
                       Scene const &scene)
{
    // Alloue les deux images qui seront sauvegardées à la fin du programme.
    Image colorImage(scene.resolution[0], scene.resolution[1]);
    Image depthImage(scene.resolution[0], scene.resolution[1]);
    
    // Crée le zBuffer.
    double *zBuffer = new double[scene.resolution[0] * scene.resolution[1]];
    for(int i = 0; i < scene.resolution[0] * scene.resolution[1]; i++) {
        zBuffer[i] = DBL_MAX;
    }

	// On utilise le constructeur qui copie à partir d'un Vector donné
	const auto cameraPosition = Vector(scene.camera.position);
	// Système de coordonnées [u, v, w]
	const auto w = Vector(scene.camera.center - cameraPosition).normalized();
	const Vector u = w.cross(scene.camera.up).normalized();
	const Vector v = u.cross(w).normalized();

	// On veut le coin inférieur droit de l'écran avec
	// O = C + (d * w) + (l * u) + (b * v)
    const double d = 1.0;
	const double t = d * tan(deg2rad(scene.camera.fovy / 2.0));
	const double r = t * scene.camera.aspect;
	const Vector lbCorner = cameraPosition + (d * w) - (r * u) - (t * v);

    // Itère sur tous les pixels de l'image.
    for(int y = 0; y < scene.resolution[1]; y++) {
        for(int x = 0; x < scene.resolution[0]; x++) {
            // Génère le rayon approprié pour ce pixel.
			Ray ray;
			if (scene.objects.empty())
			{
				// Pas d'objet dans la scène --> on rend la scène par défaut.
				// Pour celle-ci, le plan de vue est à z = 640 avec une largeur et une hauteur toutes deux à 640 pixels.
				ray = Ray(scene.camera.position, (Vector(-320, -320, 640) + Vector(x + 0.5, y + 0.5, 0) - scene.camera.position).normalized());
			}
			else
			{
				// Le pixel sur le plan de l'écran nous permet de trouver la direction du rayon
				const double deltaU = (2.0 * r) / scene.resolution[0];
				const double deltaV = (2.0 * t) / scene.resolution[1];
				const Vector Pij = lbCorner + (x + 0.5) * deltaU * u + (y + 0.5) * deltaV * v;
				const Vector direction = (Pij - cameraPosition).normalized();

				ray = Ray(cameraPosition, direction);
			}

            // Initialise la profondeur de récursivité du rayon.
            int rayDepth = 0;
           
            // Notre lancer de rayons récursif calculera la couleur et la 'z' profondeur.
            Vector color;

            // Ceci devrait être la profondeur maximum, correspondant à l'arrière-plan.
            // NOTE : Ceci suppose que la direction du rayon est de longueur unitaire (normalisée)
			//        et que l'origine du rayon est à la position de la caméra.
            double depth = scene.camera.zFar;

            // Calcule la valeur du pixel en lançant le rayon dans la scène.
            trace(ray, rayDepth, scene, color, depth);

            // Test de profondeur
            if(depth >= scene.camera.zNear && depth <= scene.camera.zFar && 
                depth < zBuffer[x + y*scene.resolution[0]]) {
                zBuffer[x + y*scene.resolution[0]] = depth;

                // Met à jour la couleur de l'image (et sa profondeur)
                colorImage.setPixel(x, y, color);
                depthImage.setPixel(x, y, (depth-scene.camera.zNear) / 
                                        (scene.camera.zFar-scene.camera.zNear));
            }
        }
		// Affiche les informations de l'étape
		if (y % 100 == 0)
		{
			printf("Row %d pixels finished.\n", y);
		}
    }

	// Sauvegarde l'image
    colorImage.writePNG(filename);
	depthImage.writePNG(depth_filename);

	printf("Ray tracing finished with images saved.\n");

    delete[] zBuffer;
}


bool Raytracer::trace(Ray const &ray, 
                 int &rayDepth,
                 Scene const &scene,
                 Vector &outColor, double &depth)
{
    // Incrémente la profondeur du rayon.
    rayDepth++;

    // — Itérer sur tous les objets en appelant Object::intersect.
    // — Ne pas accepter les intersections plus lointaines que la profondeur donnée.
    // — Appeler Raytracer::shade avec l'intersection la plus proche.
    // — Renvoyer true ssi le rayon intersecte un objet.
	if (scene.objects.empty())
	{
		// Pas d'objet dans la scène --> on rend la scène par défaut :
		// Par défaut, un cube est centré en (0, 0, 1280 + 160) avec une longueur de côté de 320, juste en face de la caméra.
		// Test d'intersection :
		double x = 1280 / ray.direction[2] * ray.direction[0] + ray.origin[0];
		double y = 1280 / ray.direction[2] * ray.direction[1] + ray.origin[1];
		if ((x <= 160) && (x >= -160) && (y <= 160) && (y >= -160))
		{
			// S'il y a intersection :
			Material m; m.emission = Vector(16.0, 0, 0); m.reflect = 0; // seulement pour le matériau par défaut ; vous devrez utiliser le matériau de l'objet intersecté
			Intersection intersection;	// seulement par défaut ; vous devrez passer l'intersection trouvée par l'appel à Object::intersect()
			outColor = shade(ray, rayDepth, intersection, m, scene);
			depth = 1280;	// la profondeur devrait être mise à jour dans la méthode Object::intersect()
		}
	}
	else
	{
		Intersection hit;
		hit.depth = depth;
		const Object* closestObject = nullptr;

		// On va checker si hit.depth est plus grand que la nouvelle distance calculée
		// Sinon, on retourne false donc on update pas closestObject
		for (auto& object: scene.objects)
		{
			if (object->intersect(ray, hit))
			{
				closestObject = object;
			}
		}

		// On évite de tracer le rayon si aucune intersection
		if (closestObject != nullptr)
		{
			outColor = shade(ray, rayDepth, hit, closestObject->material, scene);
			depth = hit.depth;
		}
	}

    // Décrémente la profondeur du rayon.
    rayDepth--;

    return false; 
}


Vector Raytracer::shade(Ray const &ray,
                 int &rayDepth,
                 Intersection const &intersection,
                 Material const &material,
                 Scene const &scene)
{
    // – Itérer sur toutes les sources de lumières, calculant les contributions ambiant/diffuse/speculaire
    // – Utiliser les rayons d'ombre pour déterminer les ombres
    // – Intégrer la contribution de chaque lumière
    // – Inclure l'émission du matériau de la surface, s'il y a lieu
    // – Appeler Raytracer::trace pour les couleurs de reflection/refraction
    // Ne pas réfléchir/réfracter si la profondeur de récursion maximum du rayon a été atteinte !
	//!!! NOTE UTILE : facteur d'atténuation = 1.0 / (a0 + a1 * d + a2 * d * d)..., la lumière ambiante ne s'atténue pas, ni n'est affectée par les ombres
	//!!! NOTE UTILE : n'acceptez pas les intersection des rayons d'ombre qui sont plus loin que la position de la lumière
	//!!! NOTE UTILE : pour chaque type de rayon, i.e. rayon d'ombre, rayon reflechi, et rayon primaire, les profondeurs maximales sont différentes
	Vector diffuse(0);
	Vector ambient(0);
	Vector specular(0);
	for (auto lightIter = scene.lights.begin(); lightIter != scene.lights.end(); lightIter++)
	{
		Intersection shadowHit;
		bool inShadow = false;

		const double SHADOW_ACNE_EPS = 1e-4;
		Vector lightVec = (lightIter->position - intersection.position);
		Vector lightDir = lightVec.normalized();
		const auto shadowRay = Ray(intersection.position + SHADOW_ACNE_EPS * intersection.normal, lightDir);

		// On regarde si un seul objet bloque le rayon d'ombrage
		for (auto& object : scene.objects)
		{
			shadowHit.depth = lightVec.length();
			if (object->intersect(shadowRay, shadowHit))
			{
				inShadow = true;
				break;
			}
		}

		if (!inShadow)
		{
			// Ombrage de Blinn-Phong
			Vector n = intersection.normal;
			Vector l = lightDir;
			Vector v = -ray.direction.normalized();
			Vector h = (l + v).normalized();

			const double diff = max(n.dot(l), 0.0);
			double spec = 0.0;
			if (diff > 0.0)
			{
				spec = pow(max(h.dot(n), 0.0), material.shininess);
			}

			const double a0 = lightIter->attenuation[0];
			const double a1 = lightIter->attenuation[1];
			const double a2 = lightIter->attenuation[2];
			const double attenuation = 1.0 / (a0 + (a1 * lightVec.length()) + (a2 * pow(lightVec.length(), 2.0)));

			diffuse += (attenuation * lightIter->diffuse) * (diff * material.diffuse);
			specular += (attenuation * lightIter->specular) * (spec * material.specular);
		}

		ambient += lightIter->ambient * material.ambient;
	}

	Vector reflectedLight(0);
	if ((!(ABS_FLOAT(material.reflect) < 1e-6)) && (rayDepth < MAX_RAY_RECURSION))
	{
		const double REFLECTION_EPS = 1e-4;
		// Formule de réflexion : R = D - 2 * (D dot N) * N
		// où D est la direction du rayon incident et N est la normale
		Vector reflectDir = ray.direction - 2.0 * ray.direction.dot(intersection.normal) * intersection.normal;
		reflectDir = reflectDir.normalized();

		// Créer le rayon réfléchi
		Ray reflectedRay(intersection.position + REFLECTION_EPS * intersection.normal, reflectDir);

		double reflectedDepth = 100.0; // soyons fous

		// Tracer le rayon réfléchi récursivement
		trace(reflectedRay, rayDepth, scene, reflectedLight, reflectedDepth);
	}

	return material.emission + ambient + diffuse + specular + material.reflect * reflectedLight;
}