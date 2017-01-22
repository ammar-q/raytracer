/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
            
    // Transform the ray to object space
    Point3D m_origin(worldToModel * ray.origin);
    Vector3D m_dir(worldToModel * ray.dir);
    
    
    Point3D p(0, 0, 0);
    Vector3D n(0, 0, 1);
    
    if (n.dot(m_dir) != 0) {
        // Ignore no intersection or infinite intersection cases

        double t_value = n.dot(p - m_origin)/ n.dot(m_dir);
        Point3D i(m_origin + t_value* m_dir);
        if (fabs(i[0]) <= 0.5 + EPSILON && fabs(i[1]) <= 0.5 + EPSILON && t_value > EPSILON && (ray.intersection.none || ray.intersection.t_value > t_value)) {
            // If current intersection is closer than previous intersection
            ray.intersection.t_value = t_value;
            ray.intersection.point = ray[t_value];
            ray.intersection.normal = transNorm( worldToModel, n * different_direction(n, m_dir));
            ray.intersection.normal.normalize();
            ray.intersection.none = false;
            return true;
        }  
    }

	return false;
}

bool UnitSquare::within( Point3D point, const Matrix4x4 worldToModel) {
    Point3D m_point(worldToModel * point);
    
    if (fabs(m_point[0]) <= 0.5 + EPSILON*EPSILON && fabs(m_point[1]) <= 0.5 + EPSILON*EPSILON && fabs(m_point[2] - 1) <= EPSILON*EPSILON) {
        return true;
    }
    return false;
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
    // Transform the ray to object space
    Point3D m_origin(worldToModel * ray.origin);
    Vector3D m_dir(worldToModel * ray.dir);
    
    Vector3D m_dir_normalized(m_dir);
    m_dir_normalized.normalize();
    
    Point3D center(0,0,0);
    
    Vector3D u(center-m_origin);
    Vector3D v(u.project(m_dir)); 
    
    double k_squared = 1 - u.dot(u) + v.dot(v);
    if (k_squared > EPSILON) {
        double k = sqrt(k_squared);
        Vector3D v_minus_k(v - k*m_dir_normalized);
        Vector3D v_plus_k(v + k*m_dir_normalized);

        double t_value;
        Point3D i;
        
        if (v_minus_k.length()/m_dir.length() * same_direction(v_minus_k, m_dir) > EPSILON) {
            t_value = v_minus_k.length()/m_dir.length() * same_direction(v_minus_k, m_dir);
            i = Point3D(m_origin + v_minus_k);
        }
        else if (v_plus_k.length()/m_dir.length() * same_direction(v_plus_k, m_dir)  >  EPSILON) {
            t_value = v_plus_k.length()/m_dir.length() * same_direction(v_plus_k, m_dir);
            i = Point3D(m_origin + v_plus_k);
        } else{
            return false;
        }
        
        if (ray.intersection.none || ray.intersection.t_value > t_value) {
            // If current intersection is closer than previous intersection
                  
            Vector3D n(i-center);
            ray.intersection.t_value = t_value;
            ray.intersection.point = ray[t_value];
            ray.intersection.normal = transNorm(worldToModel, n * different_direction(n, m_dir));
            ray.intersection.normal.normalize();
            ray.intersection.none = false;
            return true;
        }
    }
	return false;
}

bool UnitSphere::within( Point3D point, const Matrix4x4 worldToModel) {
    Point3D m_point(worldToModel * point);
    Point3D center(0,0,0);
    Vector3D u(m_point - center);
    if (u.length() <= 1 + EPSILON*EPSILON){
        return true;
    }
    return false;
}



bool UnitCone::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
    // Transform the ray to object space
    Point3D m_origin(worldToModel * ray.origin);
    Vector3D m_dir(worldToModel * ray.dir);
    double ux = m_dir[0];
    double uy = m_dir[1];
    double uz = m_dir[2];
    double ox = m_origin[0];
    double oy = m_origin[1];
    double oz = m_origin[2];
    double A = uy*uy - ux*ux - uz*uz;
    double B = 2*(oy*uy - ox*ux - oz*uz);
    double C = oy*oy - ox*ox - oz*oz;
    double descrim = B*B - 4*A*C;
    double t_value;
    bool intersection = false;
    if (descrim >= 0) {
        
        //Intersection 1
        t_value = (-B - sqrt(descrim))/(2*A);
        if (within(ray[t_value], worldToModel) 
        && t_value > EPSILON &&
        (ray.intersection.none || t_value < ray.intersection.t_value)){
            ray.intersection.t_value = t_value;
            ray.intersection.point = ray[t_value];
            Point3D i(m_origin + t_value*m_dir);
            Vector3D n(i[0], 0, i[2]);
            n.normalize();
            n[1] = 1;
            ray.intersection.normal = transNorm( worldToModel, n * different_direction(n, m_dir));
            ray.intersection.normal.normalize();
            ray.intersection.none = false;
            intersection = true;
        }
        
        //Intersection 2
        t_value = (-B + sqrt(descrim))/(2*A);
        if (within(ray[t_value], worldToModel) 
        && t_value > EPSILON && 
        (ray.intersection.none || t_value < ray.intersection.t_value)) {
            ray.intersection.t_value = t_value;
            ray.intersection.point = ray[t_value];
            Point3D i(m_origin + t_value*m_dir);
            Vector3D n(i[0], 0, i[2]);
            n.normalize();
            n[1] = 1;
            ray.intersection.normal = transNorm( worldToModel, n * different_direction(n, m_dir));
            ray.intersection.normal.normalize();
            ray.intersection.none = false;
            intersection = true;
        }
        
        //Intersection Disk
        Vector3D n(0, -1, 0);
        Point3D p(0, -1, 0);
        if (n.dot(m_dir) != 0) {
            t_value = n.dot(p - m_origin)/ n.dot(m_dir);
            Point3D i(m_origin + t_value*m_dir);
            if (within(ray[t_value], worldToModel)  && t_value > EPSILON &&
            (ray.intersection.none || (t_value < ray.intersection.t_value))) {

                ray.intersection.t_value = t_value;
                ray.intersection.point = ray[t_value];
                ray.intersection.normal = transNorm( worldToModel, n * different_direction(n, m_dir));
                ray.intersection.normal.normalize();
                ray.intersection.none = false;
                intersection = true;
            }
        }
        
        if (intersection) return true;

    }
	return false;
}

bool UnitCone::within( Point3D point, const Matrix4x4 worldToModel) {
    Point3D m_point(worldToModel * point);
    double x = m_point[0];
    double y = m_point[1];
    double z = m_point[2];
    if (y*y - x*x - z*z >= -EPSILON && fabs(y + 0.5) <= 0.5+EPSILON){ 
        return true;
    }
    return false;
}
