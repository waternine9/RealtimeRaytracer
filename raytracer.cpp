#include "opencv2/opencv.hpp"
#include <math.h>
#include <random>
using namespace cv;
using namespace std;

float radians(float x)
{
    return x * (3.14159265358979323846 / 180);
}
Vec3f lightIntensity(Vec3f sunDir, Vec3f pos1, Vec3f pos2)
{
    Vec3f diff = Vec3f(pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]);
    float mag = sqrtf(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);

    diff[0] /= mag;
    diff[1] /= mag;
    diff[2] /= mag;

    float d = diff.dot(sunDir);
    if (d > 0 && d < 1) return Vec3f(d, d, d);
    else if (d > 1) return Vec3f(1, 1, 1);
    else return 0;
}
struct EssentialsSetup
{
public:

    const static int resolutionX = 200;
    const static int resolutionY = 200;

    Mat canvas = Mat::zeros(Size(resolutionX, resolutionY), CV_8UC3);
    class Ray
    {
    public:

        Vec3f pos;
        Vec3f vec;
        Vec3i col;

    };

    class Sphere
    {
    public:

        Vec3f pos;
        Vec3i col;
        int radius;

    };

    class Triangle
    {
    public:

        Vec3f p1;
        Vec3f p2;
        Vec3f p3;
        Vec3i col;
        Vec3f normal;

    };

    class Mesh
    {
    public:
        vector<Triangle> triangles; // can't believe this works
        vector<Sphere> spheres;
    };

};

EssentialsSetup Essentials;

struct WorldObjectHandlerSetup
{
public:
    vector<EssentialsSetup::Sphere> spheres;
    vector<EssentialsSetup::Triangle> triangles;
};

WorldObjectHandlerSetup ObjectHandler;
Vec3f calcNormals(EssentialsSetup::Triangle curTriangle)
{

    Vec3f v1 = Vec3f(curTriangle.p1[0] - curTriangle.p2[0], curTriangle.p1[1] - curTriangle.p2[1], curTriangle.p1[2] - curTriangle.p2[2]);
    Vec3f v2 = Vec3f(curTriangle.p2[0] - curTriangle.p3[0], curTriangle.p2[1] - curTriangle.p3[1], curTriangle.p2[2] - curTriangle.p3[2]);
    curTriangle.normal = v1.cross(v2);
    return curTriangle.normal;
}

struct WorldSettingsSetup
{
public:

    Vec3i BGcol;
    Vec3f sunDir;

};
WorldSettingsSetup Settings;
bool rayTriangleIntersection(Vec3f rayOrigin,
    Vec3f rayVector,
    EssentialsSetup::Triangle* inTriangle,
    Vec3f& outIntersectionPoint)
{
    float y = clock();
    const float EPSILON = 0.0000001;
    Vec3f vertex0 = inTriangle->p1;
    Vec3f vertex1 = inTriangle->p2;
    Vec3f vertex2 = inTriangle->p3; 
    Vec3f edge1, edge2, h, s, q;
    float a, f, u, v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = rayVector.cross(edge2);
    a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0 / a;
    s = rayOrigin - vertex0;
    u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * rayVector.dot(q);
    // cout << (y - clock()) / CLOCKS_PER_SEC << endl;
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
    {
        outIntersectionPoint = rayOrigin + rayVector * t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}
void raytrace(Vec2i pos, int width, int height)
{

    float fov, fovx, fovy;
    fov = 80;
    int step = 0;
    fovx = -(fov / 2);
    fovy = (fov / 2);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            EssentialsSetup::Ray curray;
            curray.pos = Vec3f(j + pos[1], i + pos[0], 1.0);
            curray.vec = Vec3f(-1 + curray.pos[0] / 100, -1 + curray.pos[1] / 100, 1);
            curray.col = Settings.BGcol;

            Vec3f endray = Vec3f(curray.pos[0] + (curray.vec[0] * 1000), curray.pos[1] + (curray.vec[1] * 1000), 1000);

            for (EssentialsSetup::Sphere curSphere : ObjectHandler.spheres)
            {

                if (true) 
                {

                    float A = (curray.vec * 1000).dot(curray.vec * 1000);
                    float B = 2 * curray.vec[0] * 1000 * ((j + pos[1]) - curSphere.pos[0]) + 2 * curray.vec[1] * 1000 * ((i + pos[0]) - curSphere.pos[1]) + 2 * curray.vec[2] * 1000 * (curSphere.pos[2]);
                    float C = curSphere.pos.dot(curSphere.pos) + Vec3f(j + pos[1], i + pos[0], 0).dot(Vec3f(j + pos[1], i + pos[0], 0)) + -2 * (curSphere.pos.dot(Vec3f(j + pos[1], i + pos[0], 0))) - curSphere.radius * curSphere.radius;
                    float dist = 0;
                    dist = (-B - sqrtf(B * B - 4 * A * C)) / (2 * A);
                    float discriminant = B * B - 4 * A * C;

                    if (discriminant >= 0)
                    {
                        Vec3f lums = lightIntensity(Settings.sunDir, curSphere.pos, curray.pos);
                        curray.col = Vec3i(lums[0] * curSphere.col[0], lums[1] * curSphere.col[1], lums[2] * curSphere.col[2]);

                    }
                    else
                    {
                        curray.col[1] += (i + pos[0]) / 2;
                        curray.col[2] += (i + pos[0]) / 2;
                    }
                }
                else
                {
                    curray.col[1] += (i + pos[0]) / 2;
                    curray.col[2] += (i + pos[0]) / 2;
                }
            }
            float f = clock();
            for (EssentialsSetup::Triangle curTriangle : ObjectHandler.triangles)
            {
                Vec3f ending;
                if (rayTriangleIntersection(curray.pos, curray.vec, &curTriangle, ending))
                {
                    curTriangle.normal = calcNormals(curTriangle);
                    float lums = Settings.sunDir.dot(curTriangle.normal);
                    lums *= 255;
                    lums = (int)lums;
                    curray.col = Vec3i(lums, lums, lums);
                }
            }
            // cout << (f - clock()) / CLOCKS_PER_SEC << endl;

            Vec3b& se = Essentials.canvas.at<Vec3b>(i + pos[0], j + pos[1]);
            se.val[0] = curray.col[0];
            se.val[1] = curray.col[1];
            se.val[2] = curray.col[2];
            step = 0;
            fovx += fov / EssentialsSetup::resolutionX - pos[0];
        }
        fovy -= fov / EssentialsSetup::resolutionY - pos[1];
        fovx = -(fov / 2);
    }
}
float gravityAccel = 0;
void rTopLeft()
{
    raytrace(Vec2i(0, 0), 100, 100);
}
void rTopRight()
{
    raytrace(Vec2i(100, 0), 100, 100);
}
void rBottomLeft()
{
    raytrace(Vec2i(0, 99), 100, 100);
}
void rBottomRight()
{
    raytrace(Vec2i(100, 99), 100, 100);
}

void mouseClick(int event, int x, int y, int params, void* userdata)
{
    cout << "On" << endl;
    if (event == EVENT_LBUTTONDOWN)
    {
        gravityAccel = -4;
    }
}

int main()
{
    EssentialsSetup::Sphere curSphere;
    curSphere.pos = Vec3f(40.0, 50.0, 4.0);
    curSphere.radius = 10;
    curSphere.col = Vec3i(0, 255, 255);
    ObjectHandler.spheres.push_back(curSphere);

    EssentialsSetup::Triangle curTriangle;
    curTriangle.p1 = Vec3f(80.0, 100.0, 10.0);
    curTriangle.p2 = Vec3f(40.0, 80.0, 10.0);
    curTriangle.p3 = Vec3f(160.0, 160.0, 10.0);
    curTriangle.col = Vec3i(200, 200, 200);
    ObjectHandler.triangles.push_back(curTriangle);
    
    imshow("raytracer", Essentials.canvas);
    
    setMouseCallback("raytracer", mouseClick);
    Settings.sunDir = Vec3f(-0.2, 0.2, 1);
    Settings.BGcol = Vec3i(255, 100, 100);


    while (true)
    {
        float f = clock();
        thread tl(rTopLeft), tr(rTopRight), bl(rBottomLeft), br(rBottomRight);
        tl.join();
        tr.join();
        bl.join();
        br.join();
        // resize(Essentials.canvas, Essentials.canvas, Size(400, 400), 0, 0);
        
        // resize(Essentials.canvas, outCanvas, Size(), 2, 2);
        imshow("raytracer", Essentials.canvas);
        
        if (waitKey(1) == 'x') break;

        

        gravityAccel += 0.4; 

        ObjectHandler.spheres[0].pos[1] += gravityAccel;
        cout << 1 / ((clock() - f) / CLOCKS_PER_SEC) << endl; 

        /* if (rand() % 10 + 1 > 7)
        {
            EssentialsSetup::Triangle curTriangle;
            curTriangle.p1 = Vec3f(gravityAccel * 40, 100.0, 10.0);
            curTriangle.p2 = Vec3f(40.0, gravityAccel * 80, 10.0);
            curTriangle.p3 = Vec3f(160.0, 160.0, 10.0 + gravityAccel * 40);
            curTriangle.col = Vec3i(200, 200, 200);
            ObjectHandler.triangles.push_back(curTriangle);
        } */

        if (ObjectHandler.spheres[0].pos[1] > 200 - ObjectHandler.spheres[0].radius)
        {
            ObjectHandler.spheres[0].pos[1] = 40.0;
        }

    }



    return 0;
}