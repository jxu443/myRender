#include <vector>
#include <cmath>
#include <iostream>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

/*
1. Veci for screen_coords and Vecf for world_coords, otherwise black lines around meshes
2. Every new/new[] must have a defloate/defloate[] to avoid seg fault 11(memory leak)
3. !!! is followed by big bugs I encountered (> 3hr debugging)
*/

Model *model = NULL;
// int main(int argc, char** argv) {
//     if (2==argc) {
//         model = new Model(argv[1]);
//     } else {
//         model = new Model("obj/african_head.obj");
//     }
//     TGAImage image(width, height, TGAImage::RGB);
    
//     Vec3f light_dir(0,0,-1);
//     int *zbuffer = new int[width * height];
//     for (int i=width*height; i--; zbuffer[i] = -std::numeric_limits<float>::max()); // !!! w/ this line, no neck edges + ears

//     for (int i=0; i<model->nfaces(); i++) {
//         std::vector<int> face = model->face(i);
        
//         Vec3f world_coords[3];
//         for (int j=0; j<3; j++) {
//             Vec3f v = model->vert(face[j]);
//             world_coords[j]  = v;
//         }
//         Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
//         n.normalize();
//         float intensity = n*light_dir;
//         if (intensity>0) {
//             triangle(world_coords, zbuffer, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
//         }
//     }

//     image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
//     image.write_tga_file("output.tga");
//     defloate model;
//     defloate[] zbuffer; 
//     return 0;
// }

class Matrix {
public:
	std::vector<float> m;
	size_t r,c;

	Matrix(int _r, int _c): m(_r*_c), r(_r), c(_c) {}
	float& operator()(int i,  int j);
    float operator()(int i, int j) const; // const indicates return type without declear
	static Matrix identity(int n); //Matrix::identity(3) 
	Matrix operator*(const Matrix& a);
	Matrix transpose();
    void printMatrix();

    void selfRotateX(float angle);
};

// !!! so that matrix can be both read and written 
float& Matrix::operator()(int i,  int j) {
	return m[i * c + j];
}

float Matrix::operator()(int i,  int j) const {
	return m[i * c + j];
}

Matrix Matrix::identity(int n) { // static keyword is not needed here
	Matrix E(n,n);
	for (int i =0 ; i < n; i++) {
		E.m[i * E.c + i] = 1;
	}
	return E;
}

Matrix Matrix::transpose() {
    Matrix result(r, c);
    for(int i=0; i<r; i++)
        for(int j=0; j<c; j++)
            result(j,i) = m[i * c + j];
    return result;
}

Matrix Matrix::operator*(const Matrix& a) {
    assert(c == a.r);
    Matrix result(r, a.c);
    for (int i=0; i<r; i++) {
        for (int j=0; j<a.c; j++) {
            result(i,j) = 0.f;
            for (int k=0; k<c; k++) {
                result(i,j) += m[i * c + k] * a(k,j);
            }
        }
    }
    return result;
}

void Matrix::printMatrix() {
    for (int i=0; i<r; i++) {
        for (int j=0; j<c; j++) {
            std::cout << m[i * c + j] << ' ';
        }
        std::cout << '\n';
    }
}

Matrix translate(Vec3f v) {
    Matrix m = Matrix::identity(4);
    m(0,3) = v.x;
    m(1,3) = v.y;
    m(2,3) = v.z;
    return m;
}

Matrix scale(Vec3f v) {
    Matrix m = Matrix::identity(4);
    m(0,0) = v.x;
    m(1,1) = v.y; 
    m(2,2) = v.z;
    return m;
}

Matrix scale(float factor) {
    Matrix m = Matrix::identity(4);
    m(0,0) = factor;
    m(1,1) = factor; 
    m(2,2) = factor;
    return m;
}


Matrix rotateX(float angle)
{
    const float radius = angle * PI/180;
    Matrix m = Matrix::identity(4);
    m(1,1) = cos(radius);
    m(2,1) = sin(radius);
    m(1,2) = -sin(radius);
    m(2,2) = cos(radius);
    return m;
}

Matrix rotateY(float angle)
{
    const float radius = angle * PI/180;
    Matrix m = Matrix::identity(4);
    m(0,0) = cos(radius);
    m(2,0) = -sin(radius);
    m(0,2) = sin(radius);
    m(2,2) = cos(radius);
    return m;
}

Matrix rotateZ(float angle)
{
    const float radius = angle * PI/180;
    Matrix m = Matrix::identity(4);
    m(0,0) = cos(radius);
    m(1,0) = sin(radius);
    m(0,1) = -sin(radius);
    m(1,1) = cos(radius);
    return m;
}

//unused
void Matrix::selfRotateX(float angle)
{
    const Vec3f transl(m[0*c + 3], m[1*c + 3], m[2*c + 3]);
    const Vec3f _transl = Vec3f() - transl;

    const float radius = angle * PI/180;
    Matrix rotateM = rotateX(radius);
    *this = translate(transl) * rotateM * translate(_transl) * *this;
}

// e is location (0,0,0), g is looking at (0,0,-1), t is up (0,1,0)
Matrix viewport(Vec3f e, Vec3f g, Vec3f t) {
    g = g.normalize();
    std::cout << "g after normalize is " << g.x << g.y << g.z << '\n';
    t = t.normalize();
    Matrix Rview(4,4);
    Vec3f zerov(0,0,0); //Vec3f zerov()        error??? 
    Vec3f gxt = g ^ t; 
    std::vector<Vec3f> vec;
    vec.push_back(gxt);
    vec.push_back(t);
    vec.push_back( zerov-g);

    for (int i = 0; i < 3; i++) {
        Rview(i,0) = vec[i].x;
        Rview(i,1) = vec[i].y;
        Rview(i,2) = vec[i].z;
    }
    Rview(3,3) = 1;

    Matrix Tview = translate(zerov-e);
    return Rview*Tview;
}

Matrix ortho(float fov, float near, float far) {
    fov = fov * PI/180;
    const float aspectRatio = width / height;
    float top = tan(fov/2) * abs(near);
    float bottom = -top;
    float right = top * aspectRatio;
    float left = -right;

    Matrix scaleM = scale(Vec3f(2/(right-left), 2/(top-bottom), 2/(near- far)));  
    Matrix transM = translate(Vec3f(-(left+right)/2, -(top+bottom)/2, -(near+far)/2));
    return scaleM * transM;
}

Matrix persp(float fov, float near, float far) {
    Matrix pToO(4,4);
    pToO(0,0) = near;
    pToO(1,1) = near;
    pToO(2,2) = near+far;
    pToO(2,3) = -near*far;
    pToO(3,2) = 1;
    
    Matrix orthoM = ortho(fov, near, far);
    return orthoM * pToO;
}

Vec3f m2v(Matrix m) {
    if (m(3,0) == 0) {
        std::cout << "m(3,0) is 0 " << m.r << m.c << '\n';
        m(3,0) = 0.001;
    }
    return Vec3f(m(0,0)/m(3,0), m(1,0)/m(3,0), m(2,0)/m(3,0));
}

Matrix v2m(Vec3f v) {
    Matrix m(4, 1);
    m(0,0) = v.x;
    m(1,0) = v.y;
    m(2,0) = v.z;
    m(3,0) = 1.f;
    return m;
}

//lesson4: model, view, projection
int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/cube.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);
    
    Vec3f globalUp(0,1,0);
    Vec3f e(0,0,5);
    Vec3f g = Vec3f() - e;
    std::cout << "g is " << g.x << g.y << g.z << '\n';
    Vec3f t = (g ^ globalUp).normalize();
    t = t ^ g; // cross product is not commutative, use right-hand rule
    std::cout << "t is " << t.x << t.y << t.z << '\n';

    Matrix VP = viewport(e,g,t);
    //std::cout << "viewport matrix is" << '\n';
    //VP.printMatrix();
    Matrix Proj = ortho(90, -1, -4);
    Matrix transf = Proj * VP;
    std::cout << "transf matrix is" << '\n';
    transf.printMatrix();

    Matrix Projj = persp(90, -1, -4);
    Matrix transff = Projj * VP;
    std::cout << "transff matrix is" << '\n';
    transff.printMatrix();
    
    Vec3f screenCenter(width/2,height/2,0);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        for (int j=0; j<(int)face.size(); j++) {
            Vec3f wp0 = model->vert(face[j]);
            Vec3f wp1 = model->vert(face[(j+1)%face.size()]);

            { // draw the original modelcc
                Vec3f sp0 = m2v(transf*v2m(wp0))*(width/10) + screenCenter;
                Vec3f sp1 = m2v(transf*v2m(wp1))*(width/10) + screenCenter;
                //std::cout << "sps are " << sp0.x << ' ' << sp0.y << '\n';
                line(sp0, sp1, image, white);
            }
        }
    }
    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
