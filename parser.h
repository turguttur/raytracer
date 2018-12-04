#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>
#include <cmath>
#include <limits>

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    const float kInf = std::numeric_limits<float>::infinity();

    struct Vec3f
    {
        float x, y, z;
        Vec3f();                                    // Empty constructor
        Vec3f(float x, float y, float z);           // Constructor
        float getLengthOfVec3f();                   // Compute the length of vector
        Vec3f normalize();                          // Compute the unit vector
        float dotProduct(const Vec3f &v);           // Compute the dot product of two vector
        Vec3f crossProduct(const Vec3f &v);         // Compute the cross product of two vector
        Vec3f multiplyWithScalar(float scalar);     // Multiply the vector with a scalar
        Vec3f operator+(const Vec3f &v);            // Add two vector
        Vec3f operator-(const Vec3f &v);            // Subtract two vector
        Vec3f operator+=(const Vec3f &v);           // += operator for vectors
        Vec3f operator*(const Vec3f &v);            // Elementwise multiplication of two vectors
    };

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Ray
    {
        Vec3f o;
        Vec3f d;
        Ray(Vec3f origin, Vec3f direction);    
    };

    struct Object
    {
        int objIndex;
        int meshIndex;
        int objType;
        int materialID;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string& filepath);
        Ray getRayEquationOfPixel(int cam, int y, int x);
        bool isRayIntersectWithObject(Ray ray, float &tmin, Object &object);
        bool isShadowRayIntersectWithObject(Ray shadowRay, float lightDist);
        Vec3f computeColor(Ray ray, Object object, float tmin, int &currentDepth);
    };
}

#endif
