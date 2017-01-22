/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include <algorithm>
#include "light_source.h" 

void PointLight::shade( Ray3D& ray, double unblocked) {
    Vector3D n(ray.intersection.normal);
    n.normalize();
    
    Point3D p(ray.intersection.point);
    
    Vector3D l(_pos - p);
    l.normalize();
    
    // Calculate intensity for diffuse
    double I_d = std::max(l.cos_ang(n), 0.0);
    
    Vector3D r(l.reflect_about(n));
    // Calculate intensity for specular
    double I_s = (r.cos_ang(-ray.dir) >= 0) ? pow(r.cos_ang(-ray.dir), ray.intersection.mat->specular_exp): 0.0;
    
	Colour ambient = _col_ambient * ray.intersection.mat->ambient;
	Colour diffuse = unblocked * I_d * (_col_diffuse * ray.intersection.mat->diffuse);
	Colour specular =  unblocked * I_s * (_col_specular * ray.intersection.mat->specular);
    
    // Add the new components to current ray colour
    ray.col = ray.col + ambient + diffuse + specular;
    
    // Clamp the colour values
    ray.col.clamp();
}

void AreaLight::shade( Ray3D& ray, double unblocked) {
    PointLight P(_pos, _col_ambient);
    P.shade(ray, unblocked);
}
