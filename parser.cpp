#include "parser.h"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <iostream>

using namespace std;
//#######################################################################################################################################################################
parser::Vec3f::Vec3f() {
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
}

parser::Vec3f::Vec3f(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

float parser::Vec3f::getLengthOfVec3f() {
    return sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2));
}

parser::Vec3f parser::Vec3f::normalize() {
    float vecLength = sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2));
    this->x /= vecLength;
    this->y /= vecLength;
    this->z /= vecLength;
    return *this;
}

float parser::Vec3f::dotProduct(const Vec3f &v) {
    return ((this->x * v.x) + (this->y * v.y) + (this->z * v.z));
}

parser::Vec3f parser::Vec3f::crossProduct(const Vec3f &v) {
    Vec3f vec;
    vec.x = (this->y * v.z) - (this->z * v.y);
    vec.y = (this->z * v.x) - (this->x * v.z);
    vec.z = (this->x * v.y) - (this->y * v.x);
    return vec;
}

parser::Vec3f parser::Vec3f::multiplyWithScalar(float scalar) {
    Vec3f vec;
    vec.x = this->x * scalar;
    vec.y = this->y * scalar;
    vec.z = this->z * scalar;
    return vec;
}

parser::Vec3f parser::Vec3f::operator+(const Vec3f &v) {
    Vec3f vec;
    vec.x = this->x + v.x;
    vec.y = this->y + v.y;
    vec.z = this->z + v.z;
    return vec;
}

parser::Vec3f parser::Vec3f::operator-(const Vec3f &v) {
    Vec3f vec;
    vec.x = this->x - v.x;
    vec.y = this->y - v.y;
    vec.z = this->z - v.z;
    return vec;
}

parser::Vec3f parser::Vec3f::operator+=(const Vec3f &v) {
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    return *this;
}

parser::Vec3f parser::Vec3f::operator*(const Vec3f &v) {
    Vec3f vec;
    vec.x = this->x * v.x;
    vec.y = this->y * v.y;
    vec.z = this->z * v.z;
    return vec;
}
//#######################################################################################################################################################################
//#######################################################################################################################################################################
parser::Ray::Ray(Vec3f origin, Vec3f direction) {
    this->o = origin;
    this->d = direction;
}
//#######################################################################################################################################################################
//#######################################################################################################################################################################
parser::Ray parser::Scene::getRayEquationOfPixel(int cam, int y, int x) {
    Vec3f e = this->cameras[cam].position;              // Position vector of the camera
    Vec3f g = this->cameras[cam].gaze.normalize();      // Gaze vector of the camera
    Vec3f v = this->cameras[cam].up.normalize();        // Up vector of the camera
    Vec3f u = g.crossProduct(v).normalize();            // Right vector of the camera

    Vec4f imgPlaneBorders = this->cameras[cam].near_plane;
    float l = imgPlaneBorders.x;    // Left of the image plane
    float r = imgPlaneBorders.y;    // Right of the image plane
    float b = imgPlaneBorders.z;    // Bottom of the image plane
    float t = imgPlaneBorders.w;    // Top of the image plane

    float dist = this->cameras[cam].near_distance;                          // Distance between image plane and the camera
    int nx = this->cameras[cam].image_width;                                // Width of the image plane
    int ny = this->cameras[cam].image_height;                               // Height of the image plane
    Vec3f m = e + g.multiplyWithScalar(dist);                               // Middle point of the image plane
    Vec3f q = m + u.multiplyWithScalar(l) + v.multiplyWithScalar(t);        // Left up corner of the image plane

    float su = ((r - l)*(x + 0.5f)) / nx;                                   // How corresponding pixel far away from q in x-axis
    float sv = ((t - b)*(y + 0.5f)) / ny;                                   // How corresponding pixel far away from q in y-axis
    Vec3f s  = q + u.multiplyWithScalar(su) - v.multiplyWithScalar(sv);     // Position of the corresponding pixel 
    Vec3f d = (s - e).normalize();                                          // Direction vector of the ray        
    return Ray(e, d);     
}

bool parser::Scene::isRayIntersectWithObject(Ray ray, float &tmin, Object &object) {
    bool isIntersect = false;
    // FOR SPHERES ----------------------------------------------------------------------------------------------------------------------------
    for (int s = 0; s < this->spheres.size(); s++) {
        Vec3f c = this->vertex_data[this->spheres[s].center_vertex_id - 1];     // Center of the sphere
        float r = this->spheres[s].radius;                                      // Radius of the sphere

        Vec3f o = ray.o;    // Origin vector of the ray
        Vec3f d = ray.d;    // Direction vector of the ray

        float A = d.dotProduct(d);
        float B = 2*(d.dotProduct(o - c));
        float C = (o - c).dotProduct(o - c) - pow(r, 2);
        float disc = sqrt(pow(B, 2) - 4*A*C);       // Discriminant of the sphere equation
        if (disc >= 0) {
            float t0 = (-1 * B + disc) / (2 * A);
            float t1 = (-1 * B - disc) / (2 * A);
            float tCurrent= (t0 < t1) ? t0 : t1;
            if (tCurrent < tmin && tCurrent > 0.0f) {
                tmin = tCurrent;
                object.objIndex = s;
                object.meshIndex = -1;
                object.objType = 0;
                object.materialID = this->spheres[s].material_id;
                isIntersect = true;
            }
        }
    }

    // FOR TRIANGLES --------------------------------------------------------------------------------------------------------------------------
    for (int t = 0; t < this->triangles.size(); t++) {
        Vec3f v0 = this->vertex_data[this->triangles[t].indices.v0_id - 1];
        Vec3f v1 = this->vertex_data[this->triangles[t].indices.v1_id - 1];
        Vec3f v2 = this->vertex_data[this->triangles[t].indices.v2_id - 1];
        Vec3f A = v1 - v0;
        Vec3f B = v2 - v0; 

        float a = -(A.x);
        float b = -(A.y);
        float c = -(A.z);

        float d = -(B.x);
        float e = -(B.y);
        float f = -(B.z);

        float g = ray.d.x;
        float h = ray.d.y;
        float i = ray.d.z;

        float a1 = v0.x - ray.o.x;
        float b1 = v0.y - ray.o.y;
        float c1 = v0.z - ray.o.z;

        float detBeta = a1*(e*i - h*f) + b1*(g*f - d*i) + c1*(d*h - e*g);
        float detAlfa = a*(b1*i - h*c1) + b*(g*c1 - a1*i) + c*(a1*h - b1*g);
        float detT = a*(e*c1 - b1*f) + b*(a1*f - d*c1) + c*(d*b1 - e*a1);
                    
        float detA = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
                    
        float beta = detBeta/detA;
        float alfa = detAlfa/detA;
        float tCurrent = detT/detA; 

        if (alfa >= 0.0f && beta >= 0.0f && (alfa + beta) <= 1.0f) {
            if (tCurrent < tmin && tCurrent > 0.0f) {
                tmin = tCurrent;
                object.objIndex = t;
                object.meshIndex = -1;
                object.objType = 1;
                object.materialID = this->triangles[t].material_id;
                isIntersect = true;
            }
        }
    }

    // FOR MESHES -----------------------------------------------------------------------------------------------------------------------------
    for (int m = 0; m < this->meshes.size(); m++) {
        for (int t = 0; t < this->meshes[m].faces.size(); t++) {
            parser::Face currFace = this->meshes[m].faces[t];
            Vec3f v0 = this->vertex_data[currFace.v0_id - 1];
            Vec3f v1 = this->vertex_data[currFace.v1_id - 1];
            Vec3f v2 = this->vertex_data[currFace.v2_id - 1];
            Vec3f A = v1 - v0;
            Vec3f B = v2 - v0;

            float a = -(A.x);
            float b = -(A.y);
            float c = -(A.z);

            float d = -(B.x);
            float e = -(B.y);
            float f = -(B.z);

            float g = ray.d.x;
            float h = ray.d.y;
            float i = ray.d.z;

            float a1 = v0.x - ray.o.x;
            float b1 = v0.y - ray.o.y;
            float c1 = v0.z - ray.o.z;

            float detBeta = a1*(e*i - h*f) + b1*(g*f - d*i) + c1*(d*h - e*g);
            float detAlfa = a*(b1*i - h*c1) + b*(g*c1 - a1*i) + c*(a1*h - b1*g);
            float detT = a*(e*c1 - b1*f) + b*(a1*f - d*c1) + c*(d*b1 - e*a1);
                        
            float detA = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
                        
            float beta = detBeta/detA;
            float alfa = detAlfa/detA;
            float tCurrent = detT/detA; 

            if (alfa >= 0.0f && beta >= 0.0f && (alfa + beta) <= 1.0f) {
                if (tCurrent < tmin && tCurrent > 0.0f) {
                    tmin = tCurrent;
                    object.objIndex = m;
                    object.meshIndex = t;
                    object.objType = 2;
                    object.materialID = this->meshes[m].material_id;
                    isIntersect = true;
                }
            }
        }
    }
    return isIntersect;
}

bool parser::Scene::isShadowRayIntersectWithObject(Ray shadowRay, float lightDist) {
    // FOR SPHERES ----------------------------------------------------------------------------------------------------------------------------
    for (int s = 0; s < this->spheres.size(); s++) {
        Vec3f c = this->vertex_data[this->spheres[s].center_vertex_id - 1];     // Center of the sphere
        float r = this->spheres[s].radius;                                      // Radius of the sphere

        Vec3f o = shadowRay.o;    // Origin vector of the ray
        Vec3f d = shadowRay.d;    // Direction vector of the ray

        float A = d.dotProduct(d);
        float B = 2*(d.dotProduct(o - c));
        float C = (o - c).dotProduct(o - c) - pow(r, 2);
        float disc = sqrt(pow(B, 2) - 4*A*C);       // Discriminant of the sphere equation
        if (disc >= 0) {
            float t0 = (-1 * B + disc) / (2 * A);
            float t1 = (-1 * B - disc) / (2 * A);
            float tCurrent= (t0 < t1) ? t0 : t1;
            if (tCurrent < lightDist && tCurrent > 0.0f) {
                return true;
            }
        }
    }

    // FOR TRIANGLES --------------------------------------------------------------------------------------------------------------------------
    for (int t = 0; t < this->triangles.size(); t++) {
        Vec3f v0 = this->vertex_data[this->triangles[t].indices.v0_id - 1];
        Vec3f v1 = this->vertex_data[this->triangles[t].indices.v1_id - 1];
        Vec3f v2 = this->vertex_data[this->triangles[t].indices.v2_id - 1];
        Vec3f A = v1 - v0;
        Vec3f B = v2 - v0; 

        float a = -(A.x);
        float b = -(A.y);
        float c = -(A.z);

        float d = -(B.x);
        float e = -(B.y);
        float f = -(B.z);

        float g = shadowRay.d.x;
        float h = shadowRay.d.y;
        float i = shadowRay.d.z;

        float a1 = v0.x - shadowRay.o.x;
        float b1 = v0.y - shadowRay.o.y;
        float c1 = v0.z - shadowRay.o.z;

        float detBeta = a1*(e*i - h*f) + b1*(g*f - d*i) + c1*(d*h - e*g);
        float detAlfa = a*(b1*i - h*c1) + b*(g*c1 - a1*i) + c*(a1*h - b1*g);
        float detT = a*(e*c1 - b1*f) + b*(a1*f - d*c1) + c*(d*b1 - e*a1);
                    
        float detA = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
                    
        float beta = detBeta/detA;
        float alfa = detAlfa/detA;
        float tCurrent = detT/detA; 

        if (alfa >= 0.0f && beta >= 0.0f && (alfa + beta) <= 1.0f) {
            if (tCurrent < lightDist && tCurrent > 0.0f) {
                return true;
            }
        }
    }

    // FOR MESHES -----------------------------------------------------------------------------------------------------------------------------
    for (int m = 0; m < this->meshes.size(); m++) {
        for (int t = 0; t < this->meshes[m].faces.size(); t++) {
            parser::Face currFace = this->meshes[m].faces[t];
            Vec3f v0 = this->vertex_data[currFace.v0_id - 1];
            Vec3f v1 = this->vertex_data[currFace.v1_id - 1];
            Vec3f v2 = this->vertex_data[currFace.v2_id - 1];
            Vec3f A = v1 - v0;
            Vec3f B = v2 - v0;

            float a = -(A.x);
            float b = -(A.y);
            float c = -(A.z);

            float d = -(B.x);
            float e = -(B.y);
            float f = -(B.z);

            float g = shadowRay.d.x;
            float h = shadowRay.d.y;
            float i = shadowRay.d.z;

            float a1 = v0.x - shadowRay.o.x;
            float b1 = v0.y - shadowRay.o.y;
            float c1 = v0.z - shadowRay.o.z;

            float detBeta = a1*(e*i - h*f) + b1*(g*f - d*i) + c1*(d*h - e*g);
            float detAlfa = a*(b1*i - h*c1) + b*(g*c1 - a1*i) + c*(a1*h - b1*g);
            float detT = a*(e*c1 - b1*f) + b*(a1*f - d*c1) + c*(d*b1 - e*a1);
                        
            float detA = a*(e*i - h*f) + b*(g*f - d*i) + c*(d*h - e*g);
                        
            float beta = detBeta/detA;
            float alfa = detAlfa/detA;
            float tCurrent = detT/detA; 

            if (alfa >= 0.0f && beta >= 0.0f && (alfa + beta) <= 1.0f) {
                if (tCurrent < lightDist && tCurrent > 0.0f) {
                    return true;
                }
            }
        }
    }
    return false;
}

parser::Vec3f parser::Scene::computeColor(Ray ray, Object obj, float tmin, int &currentDepth) {
    Vec3f L;
    if (currentDepth == this->max_recursion_depth) {
        return L;
    }
    else {
        Vec3f Lm;
        Vec3f n;
        Vec3f x = ray.o + ray.d.multiplyWithScalar(tmin);
        if (obj.objType == 0) {
            n = (x - this->vertex_data[this->spheres[obj.objIndex].center_vertex_id - 1]).normalize();
        }
        else if (obj.objType == 1) {
            Vec3f v0 = this->vertex_data[this->triangles[obj.objIndex].indices.v0_id - 1];
            Vec3f v1 = this->vertex_data[this->triangles[obj.objIndex].indices.v1_id - 1];
            Vec3f v2 = this->vertex_data[this->triangles[obj.objIndex].indices.v2_id - 1];
            Vec3f A = v1 - v0;
            Vec3f B = v2 - v0;
            n = A.crossProduct(B).normalize();
        }

        else {
            Vec3f v0 = this->vertex_data[this->meshes[obj.objIndex].faces[obj.meshIndex].v0_id - 1];
            Vec3f v1 = this->vertex_data[this->meshes[obj.objIndex].faces[obj.meshIndex].v1_id - 1];
               Vec3f v2 = this->vertex_data[this->meshes[obj.objIndex].faces[obj.meshIndex].v2_id - 1];
            Vec3f A = v1 - v0;
            Vec3f B = v2 - v0;
            n = A.crossProduct(B).normalize();
        }   

        Vec3f wo = ray.d.multiplyWithScalar(-1.0f).normalize();         // Ougoing radian from object's intersection point to camera position 
        Vec3f wr = (wo.multiplyWithScalar(-1.0f) + n.multiplyWithScalar(2*(n.dotProduct(wo)))).normalize();         // Reflection vector
        Ray mirrorRay = Ray(x + wr.multiplyWithScalar(this->shadow_ray_epsilon), wr); 
        float ttmin = std::numeric_limits<float>::max();
        Object mirrorObj;
        mirrorObj.objIndex = -1;
        mirrorObj.meshIndex = -1;
        mirrorObj.objType = -1;
        mirrorObj.materialID = -1;
        if (!isRayIntersectWithObject(mirrorRay, ttmin, mirrorObj)) {
            L.x = this->background_color.x;
            L.y = this->background_color.y;
            L.z = this->background_color.z;
        }
        else {
            Lm = computeColor(mirrorRay, mirrorObj, ttmin, ++currentDepth);
            Vec3f km = this->materials[obj.materialID - 1].mirror;
            L = (km * Lm);
        }

        L += (this->ambient_light * this->materials[obj.materialID - 1].ambient);
        for (int pl = 0; pl < this->point_lights.size(); pl++) {
            Vec3f lightPosition = this->point_lights[pl].position;      // Position of the light 
            Vec3f lightIntensity = this->point_lights[pl].intensity;    // Intensity of the light
            Vec3f wi = (lightPosition - x).normalize();                 // Ingoing radiance 
            Vec3f wo = ray.d.multiplyWithScalar(-1.0f).normalize();     // Outgoing radiance 
            float r  = (lightPosition - x).getLengthOfVec3f();          // Distance from intersection point to light position

            Ray shadowRay = Ray(x + wi.multiplyWithScalar(this->shadow_ray_epsilon), wi);
            if (!isShadowRayIntersectWithObject(shadowRay, r)) {
                // DIFFUSE SHADING --------------------------------------------------------------------------------------------------------------------------------------------
                Vec3f kd = this->materials[obj.materialID - 1].diffuse;                                                 // Diffuse reflectance coefficient
                float cosTheta = max(0.0f, wi.dotProduct(n));                                                           // cos(theta) value of the diffuse shading
                Vec3f I = lightIntensity.multiplyWithScalar(pow(1.0f/r, 2));                                            // E_i(x, wi) = I / r^2
                Vec3f Ld = kd * I.multiplyWithScalar(cosTheta);                                                         // Outgoing radiance of diffuse shading

                // SPECULAR (BLINN-PHONG SHADING) -----------------------------------------------------------------------------------------------------------------------------
                Vec3f ks = this->materials[obj.materialID - 1].specular;                                                // Specular reflectance coefficient
                Vec3f h = (wi + wo).normalize();                                                                        // Half vector 
                float phongExp = this->materials[obj.materialID - 1].phong_exponent;                                    // Phong exponent of the material
                float cosAlpha = max(0.0f, n.dotProduct(h));                                                            // cos(alpha) value of the specular shading
                Vec3f Ls = ks * I.multiplyWithScalar(pow(cosAlpha, phongExp));                                          // Outgoing radiance of specular shading

                L += (Ld + Ls);
            }
        }
        return L;
    }
}
//#######################################################################################################################################################################

void parser::Scene::loadFromXml(const std::string& filepath)
{
    tinyxml2::XMLDocument file;
    std::stringstream stream;

    auto res = file.LoadFile(filepath.c_str());
    if (res)
    {
        throw std::runtime_error("Error: The xml file cannot be loaded.");
    }

    auto root = file.FirstChild();
    if (!root)
    {
        throw std::runtime_error("Error: Root is not found.");
    }

    //Get BackgroundColor
    auto element = root->FirstChildElement("BackgroundColor");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0 0 0" << std::endl;
    }
    stream >> background_color.x >> background_color.y >> background_color.z;

    //Get ShadowRayEpsilon
    element = root->FirstChildElement("ShadowRayEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> shadow_ray_epsilon;

    //Get MaxRecursionDepth
    element = root->FirstChildElement("MaxRecursionDepth");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0" << std::endl;
    }
    stream >> max_recursion_depth;

    //Get Cameras
    element = root->FirstChildElement("Cameras");
    element = element->FirstChildElement("Camera");
    Camera camera;
    while (element)
    {
        auto child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Gaze");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Up");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("NearPlane");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("NearDistance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("ImageResolution");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("ImageName");
        stream << child->GetText() << std::endl;

        stream >> camera.position.x >> camera.position.y >> camera.position.z;
        stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
        stream >> camera.up.x >> camera.up.y >> camera.up.z;
        stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
        stream >> camera.near_distance;
        stream >> camera.image_width >> camera.image_height;
        stream >> camera.image_name;

        cameras.push_back(camera);
        element = element->NextSiblingElement("Camera");
    }

    //Get Lights
    element = root->FirstChildElement("Lights");
    auto child = element->FirstChildElement("AmbientLight");
    stream << child->GetText() << std::endl;
    stream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
    element = element->FirstChildElement("PointLight");
    PointLight point_light;
    while (element)
    {
        child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Intensity");
        stream << child->GetText() << std::endl;

        stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;
        stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

        point_lights.push_back(point_light);
        element = element->NextSiblingElement("PointLight");
    }

    //Get Materials
    element = root->FirstChildElement("Materials");
    element = element->FirstChildElement("Material");
    Material material;
    while (element)
    {
        child = element->FirstChildElement("AmbientReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("DiffuseReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("SpecularReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("MirrorReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("PhongExponent");
        stream << child->GetText() << std::endl;

        stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;
        stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;
        stream >> material.specular.x >> material.specular.y >> material.specular.z;
        stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
        stream >> material.phong_exponent;

        materials.push_back(material);
        element = element->NextSiblingElement("Material");
    }

    //Get VertexData
    element = root->FirstChildElement("VertexData");
    stream << element->GetText() << std::endl;
    Vec3f vertex;
    while (!(stream >> vertex.x).eof())
    {
        stream >> vertex.y >> vertex.z;
        vertex_data.push_back(vertex);
    }
    stream.clear();

    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Mesh");
    Mesh mesh;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> mesh.material_id;

        child = element->FirstChildElement("Faces");
        stream << child->GetText() << std::endl;
        Face face;
        while (!(stream >> face.v0_id).eof())
        {
            stream >> face.v1_id >> face.v2_id;
            mesh.faces.push_back(face);
        }
        stream.clear();

        meshes.push_back(mesh);
        mesh.faces.clear();
        element = element->NextSiblingElement("Mesh");
    }
    stream.clear();

    //Get Triangles
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Triangle");
    Triangle triangle;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> triangle.material_id;

        child = element->FirstChildElement("Indices");
        stream << child->GetText() << std::endl;
        stream >> triangle.indices.v0_id >> triangle.indices.v1_id >> triangle.indices.v2_id;

        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");
    }

    //Get Spheres
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Sphere");
    Sphere sphere;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> sphere.material_id;

        child = element->FirstChildElement("Center");
        stream << child->GetText() << std::endl;
        stream >> sphere.center_vertex_id;

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> sphere.radius;

        spheres.push_back(sphere);
        element = element->NextSiblingElement("Sphere");
    }
}
