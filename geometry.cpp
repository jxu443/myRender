#include "geometry.h"
using namespace std;

void line(Vec3f p00, Vec3f p11, TGAImage &image, TGAColor color) {
    Vec2i p0(p00.x, p00.y);
    Vec2i p1(p11.x, p11.y);
    bool steep = false;
    if (std::abs(p0.x-p1.x)<std::abs(p0.y-p1.y)) {
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x>p1.x) {
        std::swap(p0, p1);
    }

    for (int x=p0.x; x<=p1.x; x++) {
        float t = (x-p0.x)/(float)(p1.x-p0.x);
        int y = p0.y*(1.-t) + p1.y*t+.5;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

/* lesson 2 point inside of triangle ? */
// version1: math
// void triangle(Vec2i *screen_pts, TGAImage &image, TGAColor color) {
//     // bounding box
//     Vec2i bboxmin(std::min(std::min(screen_pts[0].x, screen_pts[1].x), screen_pts[2].x),  
//     std::min(std::min(screen_pts[0].y, screen_pts[1].y), screen_pts[2].y)); 
    
//     Vec2i bboxmax(std::max(std::max(screen_pts[0].x, screen_pts[1].x), screen_pts[2].x),  
//     std::max(std::max(screen_pts[0].y, screen_pts[1].y), screen_pts[2].y)); 

//     // #warning "C Preprocessor got here!"
//     Vec2i P;
//     for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) { 
//         for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) { 
//             if (PointInTriangle(P, screen_pts[0], screen_pts[1], screen_pts[2])) {
//                 image.set(P.x, P.y, color); 
//             } 
//         } 
//     } 
// }

// version2: barycentric coordinate
// void triangle(Vec2i *screen_pts, TGAImage &image, TGAColor color) { 
//     // bounding box
//     Vec2i bboxmin(std::min(std::min(screen_pts[0].x, screen_pts[1].x), screen_pts[2].x),  
//     std::min(std::min(screen_pts[0].y, screen_pts[1].y), screen_pts[2].y)); 
    
//     Vec2i bboxmax(std::max(std::max(screen_pts[0].x, screen_pts[1].x), screen_pts[2].x),  
//     std::max(std::max(screen_pts[0].y, screen_pts[1].y), screen_pts[2].y)); 
//     Vec2i P; 
//     for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) { 
//         for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) { 
//             Vec3f bc_screen  = barycentric<Vec2i>(screen_pts, P); //spesify type for template method
//             if (bc_screen.x + bc_screen.y + bc_screen.z > 1.01) continue; 
//             image.set(P.x, P.y, color); 
//         } 
//     } 
// } 

/* lesson 3 z-buffer: avoid drawing of blocked pixels */
void triangle(Vec3f *world_coords, int *zbuffer, TGAImage &image, TGAColor color) { 
    Vec2i screen_pts[3]; 
    for (int j=0; j<3; j++) {
        Vec3f v = world_coords[j];
        screen_pts[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
    }
    // bounding box
    Vec2i bboxmin(std::min(std::min(screen_pts[0].x, screen_pts[1].x), screen_pts[2].x),  
    std::min(std::min(screen_pts[0].y, screen_pts[1].y), screen_pts[2].y)); 
    
    Vec2i bboxmax(std::max(std::max(screen_pts[0].x, screen_pts[1].x), screen_pts[2].x),  
    std::max(std::max(screen_pts[0].y, screen_pts[1].y), screen_pts[2].y)); 

    Vec2i P; 
    float total = 0;
    int actual = 0;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) { 
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) { 
            Vec3f bc_screen  = barycentric<Vec2i>(screen_pts, P); //spesify type for template method
            if (bc_screen.x + bc_screen.y + bc_screen.z > 1.01) continue; 

            total += 1;
            Vec3f zvec = Vec3f(world_coords[0].z, world_coords[1].z, world_coords[2].z);

            // !!! *100 to enlarge the depth number !!! 
            float depth = (zvec*100) * bc_screen; // dot
            if (zbuffer[int(P.x+P.y*width)]< depth) {
                zbuffer[int(P.x+P.y*width)] = depth;
                image.set(P.x, P.y, color);
                actual += 1;
            }
        } 
    }
}

/* 
helper functions 
*/
float triangleArea(Vec2i A, Vec2i B, Vec2i C) {
    float area = (B-A).R()  * (C-A) / 2;  
    return std::abs(area);
}

float whichSide(Vec2i p1, Vec2i p2, Vec2i p3) // three ponts
{
    /* check the sign of return */
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool PointInTriangle(Vec2i pt, Vec2i p1, Vec2i p2, Vec2i p3) // four points
{
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = whichSide(pt, p1, p2);
    d2 = whichSide(pt, p2, p3);
    d3 = whichSide(pt, p3, p1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}