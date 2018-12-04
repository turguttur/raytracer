#include <iostream>
#include <thread>
#include <ctime>
#include "parser.h"
#include "ppm.h"

using namespace std;

typedef unsigned char RGB[3];

void RenderImage(parser::Scene scene) {
    unsigned char* image;
    string image_name;
    float tmin;
    for (int c = 0; c < scene.cameras.size(); c++) {
        const int width = scene.cameras[c].image_width;
        const int height = scene.cameras[c].image_height;
        unsigned char* image = new unsigned char[width * height * 3];
        string image_name = scene.cameras[c].image_name;
        int idx = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                tmin = std::numeric_limits<float>::max();
                parser::Ray pixelRay = scene.getRayEquationOfPixel(c, y, x);
                parser::Object obj;
                obj.objIndex = -1;
                obj.meshIndex = -1;
                obj.objType = -1;
                obj.materialID = -1;
                if (scene.isRayIntersectWithObject(pixelRay, tmin, obj)) {
                    int currentDepth = 0;
                    parser::Vec3f RGB = scene.computeColor(pixelRay, obj, tmin, currentDepth);
                    if (RGB.x > 255.0f) 
                        RGB.x = 255.0f;
                    if (RGB.y > 255.0f) 
                        RGB.y = 255.0f;
                    if (RGB.z > 255.0f) 
                        RGB.z = 255.0f;
                    
                    image[idx++] = (int) RGB.x;
                    image[idx++] = (int) RGB.y;
                    image[idx++] = (int) RGB.z;
                    
                }
                else {
                    image[idx++] = scene.background_color.x;
                    image[idx++] = scene.background_color.y;
                    image[idx++] = scene.background_color.z;
                }
            }
        }
        write_ppm(image_name.c_str(), image, width, height);
    }
}

int main(int argc, char* argv[])
{
    //clock_t time;
    parser::Scene scene;
    scene.loadFromXml(argv[1]);
    RenderImage(scene);
    //thread threads[4];
    //for (int i = 0; i < 4; i++) 
    //    threads[i] = thread(RenderImage, scene);

    //for (int i = 0; i < 4; i++)
    //    threads[i].join();

    //time = clock() - time;
    //cout << "Rendering Time: " << time << endl;
    //RenderImage(scene);

    /*
    int width, height;      // Image width and height -- Image Resolution
    unsigned char* image;
    string image_name;
    float tmin;
    for (int c = 0; c < scene.cameras.size(); c++) {
        width = scene.cameras[c].image_width;
        height = scene.cameras[c].image_height;
        image = new unsigned char[width * height * 3];
        image_name = scene.cameras[c].image_name;
        int i = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                tmin = std::numeric_limits<float>::max();
                parser::Ray pixelRay = scene.getRayEquationOfPixel(c, y, x);
                parser::Object obj;
                obj.objIndex = -1;
                obj.meshIndex = -1;
                obj.objType = -1;
                obj.materialID = -1;
                if (scene.isRayIntersectWithObject(pixelRay, tmin, obj)) {
                    int currentDepth = 0;
                    parser::Vec3f RGB = scene.computeColor(pixelRay, obj, tmin, currentDepth);
                    if (RGB.x > 255.0f) 
                        RGB.x = 255.0f;
                    if (RGB.y > 255.0f) 
                        RGB.y = 255.0f;
                    if (RGB.z > 255.0f) 
                        RGB.z = 255.0f;
                    
                    image[i++] = (int) RGB.x;
                    image[i++] = (int) RGB.y;
                    image[i++] = (int) RGB.z;
                    
                }
                else {
                    image[i++] = scene.background_color.x;
                    image[i++] = scene.background_color.y;
                    image[i++] = scene.background_color.z;
                }
            }
        }
        write_ppm(image_name.c_str(), image, width, height);
    }
    */
}