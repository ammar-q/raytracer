/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implementations of util.h	

***********************************************************/

#include <cmath>
#include "util.h"

Point3D::Point3D() {
	m_data[0] = 0.0;
	m_data[1] = 0.0;
	m_data[2] = 0.0;
}

Point3D::Point3D(double x, double y, double z) { 
	m_data[0] = x;
	m_data[1] = y;
	m_data[2] = z;
}

Point3D::Point3D(const Point3D& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
}

Point3D& Point3D::operator =(const Point3D& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
	return *this;
}

double& Point3D::operator[](int i) {
	return m_data[i];
}

double Point3D::operator[](int i) const {
	return m_data[i];
}
	
Vector3D::Vector3D() {
	m_data[0] = 0.0;
	m_data[1] = 0.0;
	m_data[2] = 0.0;
}

Vector3D::Vector3D(double x, double y, double z) { 
	m_data[0] = x;
	m_data[1] = y;
	m_data[2] = z;
}

Vector3D::Vector3D(const Vector3D& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
}

Vector3D& Vector3D::operator =(const Vector3D& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
	return *this;
}

double& Vector3D::operator[](int i) {
	return m_data[i];
}
double Vector3D::operator[](int i) const {
	return m_data[i];
}

double Vector3D::length() const
{
	return sqrt(dot(*this));
}

double Vector3D::normalize() {
	double denom = 1.0;
	double x = (m_data[0] > 0.0) ? m_data[0] : -m_data[0];
	double y = (m_data[1] > 0.0) ? m_data[1] : -m_data[1];
	double z = (m_data[2] > 0.0) ? m_data[2] : -m_data[2];

	if(x > y) {
		if(x > z) {
			if(1.0 + x > 1.0) {
				y = y / x;
				z = z / x;
				denom = 1.0 / (x * sqrt(1.0 + y*y + z*z));
			}
		} else { /* z > x > y */ 
			if(1.0 + z > 1.0) {
				y = y / z;
				x = x / z;
				denom = 1.0 / (z * sqrt(1.0 + y*y + x*x));
			}
		}
	} else {
		if(y > z) {
			if(1.0 + y > 1.0) {
				z = z / y;
				x = x / y;
				denom = 1.0 / (y * sqrt(1.0 + z*z + x*x));
			}
		} else { /* x < y < z */
			if(1.0 + z > 1.0) {
				y = y / z;
				x = x / z;
				denom = 1.0 / (z * sqrt(1.0 + y*y + x*x));
			}
		}
	}

	if(1.0 + x + y + z > 1.0) {
		m_data[0] *= denom;
		m_data[1] *= denom;
		m_data[2] *= denom;
		return 1.0 / denom;
	}

	return 0.0;
}

double Vector3D::dot(const Vector3D& other) const
{
	return m_data[0]*other.m_data[0] + 
		m_data[1]*other.m_data[1] + 
		m_data[2]*other.m_data[2];
}

Vector3D Vector3D::cross(const Vector3D& other) const
{
	return Vector3D(
			m_data[1]*other[2] - m_data[2]*other[1],
			m_data[2]*other[0] - m_data[0]*other[2],
			m_data[0]*other[1] - m_data[1]*other[0]);
}

Vector3D Vector3D::project(const Vector3D& other) const
{
	return Vector3D(other*(dot(other)/other.dot(other))); 
}

Vector3D Vector3D::reflect_about(const Vector3D& other) const
{
    return Vector3D(2*project(other) - *this);
}

Vector3D Vector3D::rotate_about(const Vector3D& orth, double angle) const
{
    Vector3D orth_normalized(orth);
    orth_normalized.normalize();
    Matrix4x4 rotation;
    double c = cos(angle);
    double s = sin(angle);
    double ux = orth_normalized[0];
    double uy = orth_normalized[1];
    double uz = orth_normalized[2];
    rotation[0][0] = c + ux*ux*(1-c);
    rotation[0][1] = ux*uy*(1-c) - uz*s;
    rotation[0][2] = ux*uz*(1-c)+uy*s;
    
    rotation[1][0] = uy*ux*(1-c)+uz*s;
    rotation[1][1] = c+uy*uy*(1-c);
    rotation[1][2] = uy*uz*(1-c)-ux*s;
    
    rotation[2][0] = uz*ux*(1-c) - uy*s;
    rotation[2][1] = uz*uy*(1-c) + ux*s;
    rotation[2][2] = c + uz*uz*(1-c);
    
    return rotation * *this;
    
}

Vector3D Vector3D::noncolinear() const
{
    double X = fabs(m_data[1]);
    double Y = fabs(m_data[0]) + fabs(m_data[1]) + fabs(m_data[2]);
    return Vector3D(X, Y, Y);
}

Vector3D Vector3D::orthogonal() const
{
    return Vector3D(cross(noncolinear()));
}

double Vector3D::cos_ang(const Vector3D& other) const
{
    double ret = dot(other)/ (length() * other.length());
    if (ret > 1.0) ret = 1.0;
    if (ret < -1.0) ret = -1.0;
    return ret;
}

double Vector3D::sin_ang(const Vector3D& other) const
{
    return sin(ang(other));
}


double Vector3D::ang(const Vector3D& other) const
{
    return acos(cos_ang(other));
}

Vector3D operator *(double s, const Vector3D& v)
{
  return Vector3D(s*v[0], s*v[1], s*v[2]);
}

Vector3D operator *(const Vector3D& v, double s)
{
  return Vector3D(s*v);
}

Vector3D operator +(const Vector3D& u, const Vector3D& v)
{
  return Vector3D(u[0]+v[0], u[1]+v[1], u[2]+v[2]);
}

Point3D operator +(const Point3D& u, const Vector3D& v)
{
  return Point3D(u[0]+v[0], u[1]+v[1], u[2]+v[2]);
}

Vector3D operator -(const Point3D& u, const Point3D& v)
{
  return Vector3D(u[0]-v[0], u[1]-v[1], u[2]-v[2]);
}

Vector3D operator -(const Vector3D& u, const Vector3D& v)
{
  return Vector3D(u[0]-v[0], u[1]-v[1], u[2]-v[2]);
}

Vector3D operator -(const Vector3D& u)
{
  return Vector3D(-u[0], -u[1], -u[2]);
}

Point3D operator -(const Point3D& u, const Vector3D& v)
{
  return Point3D(u[0]-v[0], u[1]-v[1], u[2]-v[2]);
}

Vector3D cross(const Vector3D& u, const Vector3D& v) 
{
  return u.cross(v);
}

std::ostream& operator <<(std::ostream& s, const Point3D& p)
{
  return s << "p(" << p[0] << "," << p[1] << "," << p[2] << ")";
}

std::ostream& operator <<(std::ostream& s, const Vector3D& v)
{
  return s << "v(" << v[0] << "," << v[1] << "," << v[2] << ")";
}

Colour::Colour() {
	m_data[0] = 0.0;
	m_data[1] = 0.0;
	m_data[2] = 0.0;
}

Colour::Colour(double r, double g, double b) { 
	m_data[0] = r;
	m_data[1] = g;
	m_data[2] = b;
}

Colour::Colour(const Colour& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
}

Colour& Colour::operator =(const Colour& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
	return *this;
}

Colour Colour::operator *(const Colour& other) {
	return Colour(m_data[0]*other.m_data[0], 
		m_data[1]*other.m_data[1], 
		m_data[2]*other.m_data[2]);
}

double& Colour::operator[](int i) {
	return m_data[i];
}
double Colour::operator[](int i) const {
	return m_data[i];
}

void Colour::clamp() {
	for (int i = 0; i < 3; i++) {
		if (m_data[i] > 1.0) m_data[i] = 1.0; 
		if (m_data[i] < 0.0) m_data[i] = 0.0; 
	}
}

Colour operator *(double s, const Colour& c)
{
  return Colour(s*c[0], s*c[1], s*c[2]);
}

Colour operator +(const Colour& u, const Colour& v)
{
  return Colour(u[0]+v[0], u[1]+v[1], u[2]+v[2]);
}

std::ostream& operator <<(std::ostream& s, const Colour& c)
{
  return s << "c(" << c[0] << "," << c[1] << "," << c[2] << ")";
}

Vector4D::Vector4D() {
	m_data[0] = 0.0;
	m_data[1] = 0.0;
	m_data[2] = 0.0;
	m_data[3] = 0.0;
}

Vector4D::Vector4D(double w, double x, double y, double z) { 
	m_data[0] = w;
	m_data[1] = x;
	m_data[2] = y;
	m_data[3] = z;
}

Vector4D::Vector4D(const Vector4D& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
	m_data[3] = other.m_data[3];
}

Vector4D& Vector4D::operator =(const Vector4D& other) {
	m_data[0] = other.m_data[0];
	m_data[1] = other.m_data[1];
	m_data[2] = other.m_data[2];
	m_data[3] = other.m_data[3];
	return *this;
}

double& Vector4D::operator[](int i) {
	return m_data[i];
}
double Vector4D::operator[](int i) const {
	return m_data[i];
}

Matrix4x4::Matrix4x4() {
	for (int i = 0; i < 16; i++) 
		m_data[i] = 0.0; 
	m_data[0] = 1.0;
	m_data[5] = 1.0;
	m_data[10] = 1.0;
	m_data[15] = 1.0;
}

Matrix4x4::Matrix4x4(const Matrix4x4& other) {
	for (int i = 0; i < 16; i++) 
		m_data[i] = other.m_data[i]; 
}

Matrix4x4& Matrix4x4::operator=(const Matrix4x4& other) {
	for (int i = 0; i < 16; i++) 
		m_data[i] = other.m_data[i]; 
	return *this;
}

Vector4D Matrix4x4::getRow(int row) const {
	return Vector4D(m_data[4*row], m_data[4*row+1], m_data[4*row+2], 
			m_data[4*row+3]);
}

double* Matrix4x4::getRow(int row) {
	return (double*)m_data + 4*row;
}

Vector4D Matrix4x4::getColumn(int col) const {
	return Vector4D(m_data[col], m_data[4+col], m_data[8+col], 
			m_data[12+col]);
}

Vector4D Matrix4x4::operator[](int row) const {
	return getRow(row);
}

double* Matrix4x4::operator[](int row) {
	return getRow(row);
}

Matrix4x4 Matrix4x4::transpose() const {
	Matrix4x4 M; 
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			M[i][j] = (*this)[j][i]; 
		}
	}
	return M; 
}
 
Matrix4x4 operator *(const Matrix4x4& a, const Matrix4x4& b) {
	Matrix4x4 ret;

	for(size_t i = 0; i < 4; ++i) {
		Vector4D row = a.getRow(i);

		for(size_t j = 0; j < 4; ++j) {
			ret[i][j] = row[0] * b[0][j] + row[1] * b[1][j] + 
				row[2] * b[2][j] + row[3] * b[3][j];
		}
	}

	return ret;
}

Vector3D operator *(const Matrix4x4& M, const Vector3D& v) {
	return Vector3D(
			v[0] * M[0][0] + v[1] * M[0][1] + v[2] * M[0][2],
			v[0] * M[1][0] + v[1] * M[1][1] + v[2] * M[1][2],
			v[0] * M[2][0] + v[1] * M[2][1] + v[2] * M[2][2]);
}

Point3D operator *(const Matrix4x4& M, const Point3D& p) {
	return Point3D(
			p[0] * M[0][0] + p[1] * M[0][1] + p[2] * M[0][2] + M[0][3],
			p[0] * M[1][0] + p[1] * M[1][1] + p[2] * M[1][2] + M[1][3],
			p[0] * M[2][0] + p[1] * M[2][1] + p[2] * M[2][2] + M[2][3]);
}

Vector3D transNorm(const Matrix4x4& M, const Vector3D& n) {
	return Vector3D(
			n[0] * M[0][0] + n[1] * M[1][0] + n[2] * M[2][0],
			n[0] * M[0][1] + n[1] * M[1][1] + n[2] * M[2][1],
			n[0] * M[0][2] + n[1] * M[1][2] + n[2] * M[2][2]);
}

std::ostream& operator <<(std::ostream& os, const Matrix4x4& M) {
	return os << "[" << M[0][0] << " " << M[0][1] << " " 
		<< M[0][2] << " " << M[0][3] << "]" << std::endl
		<< "[" << M[1][0] << " " << M[1][1] << " " 
		<< M[1][2] << " " << M[1][3] << "]" << std::endl
		<< "[" << M[2][0] << " " << M[2][1] << " " 
		<< M[2][2] << " " << M[2][3] << "]" << std::endl
		<< "[" << M[3][0] << " " << M[3][1] << " " 
		<< M[3][2] << " " << M[3][3] << "]";
}

Point3D Ray3D::operator[](double t) 
{
    return Point3D(origin + dir*t);
}

Ray3D::Ray3D(const Ray3D& other) 
{
    origin = other.origin;
    dir = other.dir;
    intersection = other.intersection;
    col = other.col;
    strength = other.strength;
}

Ray3D Ray3D::spawn_reflection_ray()
{
    Ray3D ray = *this;
    ray.origin = intersection.point;
    ray.dir = -ray.dir;
    ray.dir = ray.dir.reflect_about(intersection.normal);
    ray.strength --;
    ray.col = Colour(0,0,0);
    ray.intersection.none = true;
    return Ray3D(ray);
}

Ray3D Ray3D::spawn_refraction_ray(double theta1, double theta2)
{
    Vector3D n(intersection.normal);
    Vector3D u(-dir);
    Vector3D up(u.cross(n));
    Ray3D ray = *this;
    ray.origin = intersection.point;
    ray.dir = u.rotate_about(up, M_PI + theta1 - theta2);
    ray.col = Colour(0,0,0);
    ray.intersection.none = true;
    return Ray3D(ray);
}

Ray3D& Ray3D::operator =(const Ray3D& other) 
{
	origin = other.origin;
    dir = other.dir;
    intersection = other.intersection;
    col = other.col;
    strength = other.strength;
	return *this;
}

Intersection& Intersection::operator =(const Intersection& other) 
{
	point = other.point;
    normal = other.normal;
    mat = other.mat;
    t_value = other.t_value;
    none = other.none;
	return *this;
}

double same_direction(Vector3D& u, Vector3D& v) {
    Vector3D w1(u-v);
    Vector3D w2(u+v);
    return (w1.length() <= w2.length())? 1.0: -1.0;
}

double different_direction(Vector3D& u, Vector3D& v) {
    return -1.0 * same_direction(u, v);
}


