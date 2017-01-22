/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		   light source classes

***********************************************************/

#include "util.h"

// Base class for a light source.  You could define different types
// of lights here, but point light is sufficient for most scenes you
// might want to render.  Different light sources shade the ray 
// differently.

enum LightType {
  _PointLight,
  _AreaLight
};

class LightSource {
public:
	virtual void shade( Ray3D& , double) = 0;
	virtual Point3D get_position() const = 0; 
    virtual LightType get_type() const = 0; 
    virtual double get_radius() const  = 0;
    virtual int get_layers() const  = 0;
    virtual int get_total() const  = 0;
};

// A point light is defined by its position in world space and its
// colour.
class PointLight : public LightSource {
public:
	PointLight( Point3D pos, Colour col ) : _pos(pos), _col_ambient(col), 
	_col_diffuse(col), _col_specular(col) {}
	PointLight( Point3D pos, Colour ambient, Colour diffuse, Colour specular ) 
	: _pos(pos), _col_ambient(ambient), _col_diffuse(diffuse), 
	_col_specular(specular) {}
	void shade( Ray3D& ray, double unblocked);
	Point3D get_position() const { return _pos; }
    LightType get_type() const {return _PointLight; }
    double get_radius() const  {return 1; };
    int get_layers() const  {return 1; };
    int get_total() const  {return 1; };
	
private:
	Point3D _pos;
	Colour _col_ambient;
	Colour _col_diffuse; 
	Colour _col_specular; 
};


// An area light is defined by its position and radius in world space and its
// colour and number of layers and total number of lights.
class AreaLight : public LightSource {
public:
	AreaLight( Point3D pos, Colour col, double radius, int layers, int total ) : _pos(pos), _col_ambient(col), 
	_col_diffuse(col), _col_specular(col), _radius(radius), _layers(layers), _total(total) {}
	AreaLight( Point3D pos, Colour ambient, Colour diffuse, Colour specular, double radius, int layers, int total ) 
	: _pos(pos), _col_ambient(ambient), _col_diffuse(diffuse), 
	_col_specular(specular), _radius(radius), _layers(layers), _total(total) {}
	void shade( Ray3D& ray, double unblocked );
	Point3D get_position() const { return _pos; }
    LightType get_type() const {return _AreaLight; }
    double get_radius() const {return _radius; }
    int get_layers() const {return _layers; }
    int get_total() const {return _total; }
	
private:
	Point3D _pos;
	Colour _col_ambient;
	Colour _col_diffuse; 
	Colour _col_specular; 
    double _radius;
    int _layers;
    int _total;
};

