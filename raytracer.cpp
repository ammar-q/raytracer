/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <algorithm>


Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}
void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
    traverseScene(node,ray,_modelToWorld,_worldToModel);
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray, const Matrix4x4& modelToWorld, const Matrix4x4& worldToModel ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	Matrix4x4 myModelToWorld = modelToWorld*node->trans;
	Matrix4x4 myWorldToModel = node->invtrans*worldToModel;
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, myWorldToModel, myModelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray, myModelToWorld,myWorldToModel);
		childPtr = childPtr->next;
	}

}

double Raytracer::lightCollectionTraversal( SceneDagNode* node, Ray3D& ray ) {
    return lightCollectionTraversal(node,ray,_modelToWorld,_worldToModel);
}

double Raytracer::lightCollectionTraversal( SceneDagNode* node, Ray3D& ray, const Matrix4x4& modelToWorld, const Matrix4x4& worldToModel ) {
	SceneDagNode *childPtr;
    double light = 1.0;
	// Applies transformation of the current node to the global
	// transformation matrices.
	Matrix4x4 myModelToWorld = modelToWorld*node->trans;
	Matrix4x4 myWorldToModel = node->invtrans*worldToModel;
	if (node->obj) {
		// Perform intersection.
        Ray3D ray_copy(ray);
		if (node->obj->intersect(ray_copy, myWorldToModel, myModelToWorld)) {
			light *= node->mat->refraction_coefficient;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		light *= lightCollectionTraversal(childPtr, ray, myModelToWorld,myWorldToModel);
		childPtr = childPtr->next;
	}
    return light;

}

double Raytracer::pathUnblocked( const Point3D& source, const Point3D& dest ) 
{
    Ray3D source_to_dest(source, dest-source);
    source_to_dest.intersection.none = false;
    source_to_dest.intersection.t_value = 1.0-EPSILON;
    return lightCollectionTraversal(_root, source_to_dest);
    
}

double Raytracer::computeRefractionIndex( SceneDagNode* node, Point3D& point ) {
    return computeRefractionIndex(node, point,_modelToWorld,_worldToModel);
}

double Raytracer::computeRefractionIndex( SceneDagNode* node, Point3D& point, const Matrix4x4& modelToWorld, const Matrix4x4& worldToModel ) {
	SceneDagNode *childPtr;

    double combined = 0;
	// Applies transformation of the current node to the global
	// transformation matrices.
	Matrix4x4 myModelToWorld = modelToWorld*node->trans;
	Matrix4x4 myWorldToModel = node->invtrans*worldToModel;
	if (node->obj) {
		// Check if point is within object 
		if (node->obj->within(point, myWorldToModel)) {
			combined += node->mat->refraction_index;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		combined += computeRefractionIndex(childPtr, point, myModelToWorld,myWorldToModel);
		childPtr = childPtr->next;
	}
    return combined;
}


void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
    ray.col = Colour(0,0,0);
	for (;;) {
		if (curLight == NULL) break;
        double unblocked = 0.0;
        
        if (curLight->light->get_type() == _PointLight){
            // Collect light intensity to intersection point
            unblocked = pathUnblocked(curLight->light->get_position(), ray.intersection.point);
        }
        
        if (curLight->light->get_type() == _AreaLight){
            int T = curLight->light->get_total();
            int L = curLight->light->get_layers();
            float r = curLight->light->get_radius()/L;
            Vector3D n(-ray.intersection.normal);
            n.normalize();
            float count = 0;
            for (int l = 1; l <= L; l++){
                int numLights = int(ceil((2*l - 1)*T/(L*L)));
                count += numLights;
                double angle = 2*M_PI / numLights;
                Vector3D delta(n.orthogonal());
                delta.normalize();
                delta = delta * r * l;
                Point3D center(curLight->light->get_position());
                for (int i = 0; i < numLights; i++){
                    // Calculate light position
                    Point3D pos = center + delta;
                    // Collect light intensity to intersection point
                    unblocked += pathUnblocked(pos, ray.intersection.point);
                    delta = delta.rotate_about(n, angle);
                }
            }
            // Average the collected light intensities
            unblocked /= count;
        }
        
        // Shade the point according to light intensity
		curLight->light->shade(ray, unblocked);
        
		curLight = curLight->next;
	}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray ) {
	Colour col(0.0, 0.0, 0.0); //Background;
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit anything.
	if (!ray.intersection.none) {
		computeShading(ray); 
		col = ray.col;  
        Ray3D reflected;
        Ray3D refracted;
       
        double rfl_c = ray.intersection.mat->reflection_coefficient;
        
        // Check if ray can be reflected
        if (rfl_c > 0 && ray.strength > 0) {
            double gl_n = ray.intersection.mat->glossy_num;
            double gl_d = ray.intersection.mat->glossy_deviation;
            Colour sum(0,0,0);
            for (int i = 0; i < int(gl_n); i++) {
                // Spawn reflection ray
                reflected = ray.spawn_reflection_ray();
                
                // Deviate the ray according to gl_d 
                Vector3D delta(reflected.dir.orthogonal());
                delta.normalize();
                double r1 = double(rand())/RAND_MAX;
                double r2 = double(rand())/RAND_MAX;
                delta = (gl_d * r2) * delta.rotate_about(reflected.dir, (2*M_PI * r1));
                reflected.dir = reflected.dir + delta;
                
                // Recursively shade reflected ray
                sum = sum + shadeRay(reflected);
            }
            // Add the appropriate amount of indirect lighting from reflection
            col = col + (rfl_c/gl_n) * sum;
        }

        double rfr_c = ray.intersection.mat->refraction_coefficient;
        
        // Check if ray can be refracted
        if (rfr_c > 0) {
            Vector3D n(ray.intersection.normal);
            Vector3D u(-ray.dir);
            
            // Find points in the two mediums
            Point3D point_in_medium1 = ray.intersection.point + 10*EPSILON * ray.intersection.normal;
            Point3D point_in_medium2 = ray.intersection.point + -10*EPSILON * ray.intersection.normal;
            
            // Calculate refraction indices
            double current_index = std::max(computeRefractionIndex(_root, point_in_medium1), 1.0);
            double dest_index = std::max(computeRefractionIndex(_root, point_in_medium2), 1.0);
            
            // Calculate angles
            double theta1 = u.ang(n);
            double theta2 = asin(sin(theta1) * current_index / dest_index);
            
            if (fabs(theta2) < M_PI/2){
                // Spawn refraction ray
                refracted = ray.spawn_refraction_ray(theta1, theta2);
                // Add the appropriate amount of indirect lighting from refraction
                col = col + rfr_c*shadeRay(refracted);
            }
            
        }
        
        // Clamp colour values
        col.clamp();
    }
    return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, int reflection_depth, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
			imagePlane[0] = (-double(width)/2 + 0.5 + j)/factor;
			imagePlane[1] = (-double(height)/2 + 0.5 + i)/factor;
			imagePlane[2] = -1;
	
			
			Ray3D ray(origin, imagePlane - origin);
            ray.origin = viewToWorld * ray.origin;
            ray.dir =  viewToWorld * ray.dir;
            ray.dir.normalize();
            ray.strength = reflection_depth;
            
			Colour col = shadeRay(ray); 
            col.clamp();
			_rbuffer[i*width+j] = int(col[0]*255);
			_gbuffer[i*width+j] = int(col[1]*255);
			_bbuffer[i*width+j] = int(col[2]*255);
		}
	}

	flushPixelBuffer(fileName);
}

void Raytracer::anti_aliased_render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, int granularity, int reflection_depth, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// image plane is at z = -1
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
            Colour col(0,0,0);
	
            double delta = 1/double(granularity);
            for (int a = 0; a < granularity; a ++){
                for (int b = 0; b < granularity; b ++){
                   double r1 = double(rand())/RAND_MAX;
                   double r2 = double(rand())/RAND_MAX;
                   imagePlane[0] = (-double(width)/2 + j + (a+r1)*delta)/factor;
                   imagePlane[1] = (-double(height)/2 + i + (b+r2)*delta )/factor;
                   imagePlane[2] = -1;
                   Ray3D ray(origin, imagePlane - origin);
                   ray.origin = viewToWorld * ray.origin;
                   ray.dir =  viewToWorld * ray.dir;
                   ray.dir.normalize();
                   ray.strength = reflection_depth;
            
			       col = col + shadeRay(ray); 
                   
                }
            }
			col = delta*delta*col;
            col.clamp();

			_rbuffer[i*width+j] = int(col[0]*255);
			_gbuffer[i*width+j] = int(col[1]*255);
			_bbuffer[i*width+j] = int(col[2]*255);
		}
	}

	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240; 

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 3);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;
	
	
	//Colour powderblue(0.690196, 0.878431, 0.901961);
	Colour aliceblue(0.941176, 0.972549, 1.0);
	//Colour lightsteelblue(0.690196, 0.768627, 0.870588);
	Material glass(Colour(0,0,0) , Colour(0.1,0.1,0.3), aliceblue, 101.2, 1.52, 0.8, 0.3, 3, 0.01);

	// Defines a material for shading.
	Material gold( Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2, 2.1, 0, 0.4 , 1, 0 );
	Material jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63), 
			Colour(0.316228, 0.316228, 0.316228), 
			12.8, 1.3, 0, 1.0, 4, 0.1);
            
    Material red( Colour(0.4, 0.1, 0.1), Colour(0.541176, 0.027451, 0.027451), 
			Colour(0.728281, 0.255802, 0.266065), 
			31.2, 3.5, 0, 0.4 , 4, 0.3);

	// Defines a point light source.
	//raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
	//			Colour(0.9, 0.9, 0.9) ) );
    // Defines an area light source
    raytracer.addLightSource( new AreaLight(Point3D(0, 0, 5), 
				Colour(0.9, 0.9, 0.9), 1.0, 4, 100 ) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &gold );
    SceneDagNode* sphere2 = raytracer.addObject( new UnitSphere(), &glass );
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
	
	// Apply some transformations to the unit square.
	double factor1[3] = { 1.0, 2.0, 1.0 };
	double factor2[3] = { 6.0, 6.0, 6.0 };
    double factor3[3] = { 1.15, 1.15, 1.15 };
	raytracer.translate(sphere, Vector3D(-1, -1, -3));	
	raytracer.rotate(sphere, 'x', -45); 
	raytracer.rotate(sphere, 'z', 45); 
    raytracer.scale(sphere, Point3D(0, 0, 0), factor1);
    
    raytracer.translate(sphere2, Vector3D(1, 1, -3.5));
    raytracer.scale(sphere2, Point3D(0,0,0), factor3);	
	

	raytracer.translate(plane, Vector3D(0, 0, -7));	
	raytracer.rotate(plane, 'z', 45); 
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);
    

	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	//raytracer.render(width, height, eye, view, up, fov, 0, "view1.bmp");
    raytracer.anti_aliased_render(width, height, eye, view, up, fov, 4, 3,  "view1.bmp");
	
	// Render it from a different point of view.
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);
	//raytracer.render(width, height, eye2, view2, up, fov, "view2.bmp");
    raytracer.anti_aliased_render(width, height, eye2, view2, up, fov, 4, 3, "view2.bmp");
	
	return 0;
}

