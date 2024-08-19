#version 330 core
// #pragma fragmentoption ARB_precision_hint_nicest

#extension GL_ARB_arrays_of_arrays : require
#extension GL_ARB_shader_storage_buffer_object : require
// #extension GL_ARB_gpu_shader_fp64 : require


#define DEV_CHUNKS_COUNT 1


// #define DEV_DDA

//or

#define DEV_AABB

#define DEV_CHILDNODE_SKIPPING
#define DEV_BACKTRACKING

in vec4 vColor;
in vec2 vTexCoord;
in vec3 vNormal;
in vec3 vFragPos;

out vec4 fragColor;
const float FOV = 1.0;

const float NoiseRenderThreshold = 0.9;

const int u_AA_type = 0;

// #define RO_INSIDE_CHUNK_DOUBLE_CHECK


uniform int DEV_BACKTRACKING_MODE = 1;



// #define EPSILON 0.000001
#define EPSILON 0.00000625
#define EPSILON_M 0.000625
#define MAX_RENDER_DISTANCE 1000


// LOD settings
#define LOD_DISTANCE_PER_LAYER 10
#define LOD_MIN_LAYER 1
#define LOD_RPM_MULTIPLIER 2

#define OCTREE_MAX_LAYER 11

const float LOD_DISTANCE_ARRAY[9] = float[9](
    32,32,32,32,32,32,32,32,100
);

#define CHUNK_SIZE 32.0
#define HALF_CHUNK_SIZE CHUNK_SIZE / 2
#define MIN_VOXEL_SIZE 0.03125
#define MIN_VOXEL_SIZE_HALF 0.03125
#define MIN_VOXEL_SIZE_QUART 0.015625
#define MAX_VOXEL_DIVIDE_COUNT 9

#define VOXELS_PER_CHUNK_EDGE 512
#define VOXELS_PER_CHUNK VOXELS_PER_CHUNK_EDGE * VOXELS_PER_CHUNK_EDGE * VOXELS_PER_CHUNK_EDGE

#define HYPOTENUSE 1.4142135624
#define M_PI 3.1415926535897932384626433832795

#define PI 3.14159265
#define TAU (2*PI)
#define PHI (sqrt(5)*0.5 + 0.5)

// #define GLOBAL_ILLUMINATION
#define LIGHT_DOT_SHADOWS
// #define PER_VOXEL_ILLUMINATION 0.0625
// #define AMBIENT_OCCLUSION

#define CAMERA_ILLUMINATION 3

// Actualy this is sun "size". 1 = 180 deg of skybox
#define SUN_BLOOM 0.005
#define LightDotMultiplier 5
// How fast shadow for not lited side appears
#define ShadowDotMultiplier 100
// Max value for "shadow darkness", used in voxel_color / SHADOW_COLOR as 1 / ShadowMaxCap
#define ShadowMaxCap 10
#define VOXEL_SIZE 0.0625


// #define VOXEL_NOISE
////    real value
#define VOXEL_NOISE_MULTIPLIER 0.025
////    debug value
// #define VOXEL_NOISE_MULTIPLIER 0.1
// this value equals noise-map pixel size to one voxel (min voxel size)
#define VOXEL_NOISE_SIZE 0.5


#define VISUALIZE_ITERATIONS 5
// #define DISTANCE_FIELD_DEBUG
// #define VOXEL_NORMAL_DEBUG
// #define DRAW_VOXEL_BORDERS



// loop iterations
#define VISUALIZE_ITERATIONS_MULTIPLIER 256.0

//memory reads
#define VISUALIZE_ITERATIONS_MULTIPLIER_2 256.0

//raybox checks
#define VISUALIZE_ITERATIONS_MULTIPLIER_3 256.0

//memread skip caues by backtracking
#define VISUALIZE_ITERATIONS_MULTIPLIER_4 256.0



uniform int VISUALIZE_ITERATIONS_MODE = 1;

#define SOFT_SHADOWS_SUN_SIZE 5;

const vec3 SUN_SOFT_SHADOWS_VEC[7] = vec3[7](
    vec3(0),
    vec3(-1,0,0),
    vec3(1,0,0),
    vec3(0,-1,0),
    vec3(0,1,0),
    vec3(0,0,-1),
    vec3(0,0,1)
    // vec3(0,0,0)
);


// States indices
#define OctFillState 24
#define OctFullfilled 25
#define OctSurrounded 26
#define OctOpacity 27
#define OctDivided 28
#define OctIsLeaf 8




// #define WIRECUBE_DEBUG
#define WIRECUBE_DISTANCE 5

// Octree nodes position offset

const vec3 OctNodeOffset[8] = vec3[8](
    vec3(1, 1, 1),
    vec3(-1, 1, 1),
    vec3(-1, -1, 1),
    vec3(1, -1, 1),
    
    vec3(1, 1, -1),
    vec3(-1, 1, -1),
    vec3(-1, -1, -1),
    vec3(1, -1, -1)
);

// #define ShowLightCalculations

// #define showLOD

// #define ChunkSize 32
// #define ChunkSizeHalf 16

uniform int NoiseRender = 0;

// uniform vec3 globalLightDirection = vec3(0.5, -0.25, -1);
uniform vec3 globalLightDirection = vec3(1, -0.5,-0.01);
// uniform vec3 globalLightDirection = vec3(0, 1, 0);
vec4 globalLightColor = vec4(1, 1 , 0.85, 1);
vec4 ambient = vec4(1, 1, 1, 1);
vec4 backgroundColor = vec4(0.5, 0.8, 0.9, 1);
vec4 voivColor = vec4(0.1, 0.1, 0.1, 1);

uniform vec2 u_resolution;
uniform float u_time = 0;

uniform int u_chunks_count = 1;

uniform vec3 u_camera_position;
uniform vec3 u_camera_forward;
uniform vec3 u_camera_right;
uniform vec3 u_camera_up;
uniform float u_mouse_wheel = 1;

uniform int u_render_variant = 1;


uniform int CHUNK_RENDER_VARIANT = 1;


uniform samplerBuffer u_tbo_tex;
uniform samplerBuffer u_tbo_tex2;

float specularStrength = 0.5;

const float toCamDistArray[5] = float[5](5, 4, 3, 2, 1);
const vec4 toCamDistArrayCol[5] = vec4[5](
    vec4(vec3(1),1),
    vec4(vec3(0.8),1),
    vec4(vec3(0.6),1),
    vec4(vec3(0.4),1),
    vec4(vec3(0.2),1)
);

const vec4 COLOR_PALETTE[9] = vec4[9](
    vec4(1, 0 ,0 , 1),      //  0   red
    vec4(0, 1, 0, 1),       //  1   green
    vec4(0, 0, 1, 1),       //  2   blue
    vec4(1, 1, 0, 1),       //  3   yellow
    vec4(1, 0, 1, 1),       //  4   pink
    vec4(0, 1, 1, 1),       //  5   aqua
    vec4(0.5, 0.5, 0.5, 1), //  6   grey
    vec4(0, 0, 0, 1),       //  7   black
    vec4(1, 1, 1, 1)        //  8   white
);

const float OCTREE_VOXEL_SIZES[21] = float[21](
    32.0, 16.0, 8.0, 4.0, 2.0, 1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125, 0.00390625, 0.001953125, 0.0009765625, 0.00048828125, 0.000244140625, 0.0001220703125, 0.00006103515625, 0.000030517578125
);


const vec3 OCTREE_DIAGONAL_DIRECTIONS[8] = vec3[8]
(
    vec3(-1,1,-1), //left top far       - + -
    vec3(1,1,-1), //right top far       + + -
    vec3(1,1,1), //right top near       + + +
    vec3(-1,1,1), //left top near       - + +

    vec3(-1,-1,-1), //left bottom far   - - -
    vec3(1,-1,-1), // right bottom far  + - -
    vec3(1,-1,1), //right bottom near   + - +
    vec3(-1,-1,1) //left bottom near    - - +
);

#define CHUNK_INFO_START_POINTER 8
const int FLOATS_PER_VOXEL = 12;

// int DefVoxelArray2[VOXELS_PER_CHUNK_EDGE, VOXELS_PER_CHUNK_EDGE, VOXELS_PER_CHUNK_EDGE];
// int DefVoxelArray[VOXELS_PER_CHUNK];

#define SSBO_LAYERS  64

layout(std430, binding = 3) buffer ssboLayout
{
    int DefVoxelArray[];
};

layout(std430, binding = 4) buffer ssboLayout2
{
    int DefVoxelArray2[];
};



struct VoxelHit{
    vec3 pos;
    vec3 hit;
    vec3 normal;
    float dist;
    float distOut;
    float voxelSize;
    vec4 color;
    bool crossed;
    int state;
};

struct Ray2VoxelHit{
    vec3 voxelCenter;
    float voxelHalfSize;
    int voxelLayer;
    int voxelIndex;
};

struct Voxel{
    vec4 color;
    int data;
    int bottomLeafsCount;
    vec3 position;
    int leafLevel;
};

struct OctreeNode{
    vec3 position;
    int offset;
};

struct Chunk{
    int offset;
    float dist;
};

// Rotate around a coordinate axis (i.e. in a plane perpendicular to that axis) by angle <a>.
// Read like this: R(p.xz, a) rotates "x towards z".
// This is fast if <a> is a compile-time constant and slower (but still practical) if not.
void pR(inout vec2 p, float a) {
	p = cos(a)*p + sin(a)*vec2(p.y, -p.x);
}

// Shortcut for 45-degrees rotation
void pR45(inout vec2 p) {
	p = (p + vec2(p.y, -p.x))*sqrt(0.5);
}

void rotate(inout vec3 p, vec3 r){
	if(r.x != 0)
		pR(p.yz, r.x);
	if(r.y != 0)
		pR(p.xz, r.y);
	if(r.z != 0)
		pR(p.xy, r.z);

	// return p;
}
void rotate(inout vec3 p, float x = M_PI * 2, float y = M_PI * 2, float z = M_PI * 2){
	pR(p.yz, x);
	pR(p.xz, y);
	pR(p.xy, z);

	// return p;
}

void xRot(inout vec3 p, float r){
	pR(p.yz, r);
}
void yRot(inout vec3 p, float r){
	pR(p.xz, r);
}
void zRot(inout vec3 p, float r){
	pR(p.xy, r);
}

vec4 invertColor(vec4 color){
    color.x = (color.x * -1) + 1;
    color.y = (color.y * -1) + 1;
    color.z = (color.z * -1) + 1;
    return color;
}

vec4 colorFromBytes(int colBytes){
    vec4 voxColor = vec4(1);
    voxColor.x = float((colBytes >> 24) & 0xff) / 255;
    voxColor.y = float((colBytes >> 16) & 0xff) / 255;
    voxColor.z = float((colBytes >> 8) & 0xff) / 255;
    voxColor.w = float((colBytes >> 0) & 0xff) / 255;

    return voxColor;
}

ivec4 BytesFromInt(int bytes){
    ivec4 voxColor = ivec4(1);
    voxColor.x = (bytes >> 24) & 0xff;
    voxColor.y = (bytes >> 16) & 0xff;
    voxColor.z = (bytes >> 8) & 0xff;
    voxColor.w = (bytes >> 0) & 0xff;

    return voxColor;
}


int getFloatBytes(float value, int index){
    int v = floatBitsToInt(value);

    return (v >> (index * 8)) & 0xff;
}
uint getUintBytes(uint v, int index){
    // int v = int(value);
    return (v >> (index * 8)) & uint(0xff);
}
uint bytesToUint(uint b1, uint b2, uint b3, uint b4){
    b1 = b1 << (3 * 8);
    b2 = b2 << (2 * 8);
    b3 = b3 << (1 * 8);
    return b1 + b2 + b3 + b4;
}

float TexelFetch1(int offset, samplerBuffer tbo = u_tbo_tex){
    int cOffset = offset % 4;
    offset = (offset - cOffset) / 4;
    return texelFetch(tbo, offset)[cOffset];
}

float GetLeafSize(uint index){
    return CHUNK_SIZE / (pow(2, index));
}

int TexelFetchByte(int offset){
    int ob = (offset % 16) % 4;
    int oi = offset % 4;
    int i = (offset - oi) / 4;

    // int cOffset = offset % 4;
    // offset = (offset - cOffset) / 4;
    int a = floatBitsToInt(texelFetch(u_tbo_tex, i - (i%4))[i % 4]);
    a = 0x5;
    int texel = ((a >> ((ob * 4))) & 0xff);

    return texel;
}

vec4 TexelFetch4(int offset){
    return texelFetch(u_tbo_tex, offset);
}

bool IsPointInside(vec3 pos, vec3 minCorn = vec3(-HALF_CHUNK_SIZE), vec3 maxCorn = vec3(HALF_CHUNK_SIZE))
{
    return pos.x <= maxCorn.x && pos.x >= minCorn.x && pos.y <= maxCorn.y && pos.y >= minCorn.y && pos.z <= maxCorn.z && pos.z >= minCorn.z;
}
bool IsPointInside_2(vec3 pos, vec3 center, vec3 bounds)
{
    pos = abs(pos - center);
    return pos.x <= bounds.x && pos.y <= bounds.y && pos.z <= bounds.z;
    // return pos.x <= maxCorn.x && pos.x >= minCorn.x && pos.y <= maxCorn.y && pos.y >= minCorn.y && pos.z <= maxCorn.z && pos.z >= minCorn.z;
}

bool PointAABB(vec3 v, vec3 bottomLeft, vec3 topRight, float eps = EPSILON) {
    vec3 s = step(bottomLeft - vec3(eps), v) - step(topRight + vec3(eps), v);
    return ( s.x * s.y * s.z) > 0; 
}

bool bitInt(int value, int bit){
	return ((value >> bit) & 1) == 1;
}

float SqrMagnitude(vec3 v){
    return v.x * v.x + v.y * v.y + v.z * v.z; 
}
float Magnitude(vec3 v){
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z); 
}

float rand(vec2 co){
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

float lengthSqr(vec3 x) {
	return dot(x, x);
}
float length(vec3 x) {
	return dot(x, x);
}

vec3 BoxNormal(vec3 dir){
    // return dir;
    vec3 d = normalize(abs(dir));
    if(d.x >= d.y && d.x >= d.z){
        return vec3(1,0,0) * sign(dir.x);
    }
    if(d.y >= d.x && d.y >= d.z){
        return vec3(0,1,0) * sign(dir.y);
    }
    // if(d.z > d.x && d.z > d.y){
    //     return vec3(0,0,1) * sign(dir.z);
    // }
    return vec3(0,0,1) * sign(dir.z);
}
vec4 ColorBlend(vec4 a, vec4 b, float at, float bt){
    float tSumm = at + bt;
    at = 1 / tSumm * at;
    bt = 1 / tSumm * bt;
    return (a * at) + (b * bt);
}

float mod289(float x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
vec4 mod289(vec4 x){return x - floor(x * (1.0 / 289.0)) * 289.0;}
vec4 perm(vec4 x){return mod289(((x * 34.0) + 1.0) * x);}

float noise3(vec3 p){
    vec3 a = floor(p);
    vec3 d = p - a;
    d = d * d * (3.0 - 2.0 * d);

    vec4 b = a.xxyy + vec4(0.0, 1.0, 0.0, 1.0);
    vec4 k1 = perm(b.xyxy);
    vec4 k2 = perm(k1.xyxy + b.zzww);

    vec4 c = k2 + a.zzzz;
    vec4 k3 = perm(c);
    vec4 k4 = perm(c + 1.0);

    vec4 o1 = fract(k3 * (1.0 / 41.0));
    vec4 o2 = fract(k4 * (1.0 / 41.0));

    vec4 o3 = o2 * d.z + o1 * (1.0 - d.z);
    vec2 o4 = o3.yw * d.x + o3.xz * (1.0 - d.x);

    return o4.y * d.y + o4.x * (1.0 - d.y);
}

//	Classic Perlin 3D Noise 
//	by Stefan Gustavson
//
vec4 permute(vec4 x){return mod(((x*34.0)+1.0)*x, 289.0);}
vec4 taylorInvSqrt(vec4 r){return 1.79284291400159 - 0.85373472095314 * r;}
vec3 fade(vec3 t) {return t*t*t*(t*(t*6.0-15.0)+10.0);}

float cnoise(vec3 P){
    vec3 Pi0 = floor(P); // Integer part for indexing
    vec3 Pi1 = Pi0 + vec3(1.0); // Integer part + 1
    Pi0 = mod(Pi0, 289.0);
    Pi1 = mod(Pi1, 289.0);
    vec3 Pf0 = fract(P); // Fractional part for interpolation
    vec3 Pf1 = Pf0 - vec3(1.0); // Fractional part - 1.0
    vec4 ix = vec4(Pi0.x, Pi1.x, Pi0.x, Pi1.x);
    vec4 iy = vec4(Pi0.yy, Pi1.yy);
    vec4 iz0 = Pi0.zzzz;
    vec4 iz1 = Pi1.zzzz;

    vec4 ixy = permute(permute(ix) + iy);
    vec4 ixy0 = permute(ixy + iz0);
    vec4 ixy1 = permute(ixy + iz1);

    vec4 gx0 = ixy0 / 7.0;
    vec4 gy0 = fract(floor(gx0) / 7.0) - 0.5;
    gx0 = fract(gx0);
    vec4 gz0 = vec4(0.5) - abs(gx0) - abs(gy0);
    vec4 sz0 = step(gz0, vec4(0.0));
    gx0 -= sz0 * (step(0.0, gx0) - 0.5);
    gy0 -= sz0 * (step(0.0, gy0) - 0.5);

    vec4 gx1 = ixy1 / 7.0;
    vec4 gy1 = fract(floor(gx1) / 7.0) - 0.5;
    gx1 = fract(gx1);
    vec4 gz1 = vec4(0.5) - abs(gx1) - abs(gy1);
    vec4 sz1 = step(gz1, vec4(0.0));
    gx1 -= sz1 * (step(0.0, gx1) - 0.5);
    gy1 -= sz1 * (step(0.0, gy1) - 0.5);

    vec3 g000 = vec3(gx0.x,gy0.x,gz0.x);
    vec3 g100 = vec3(gx0.y,gy0.y,gz0.y);
    vec3 g010 = vec3(gx0.z,gy0.z,gz0.z);
    vec3 g110 = vec3(gx0.w,gy0.w,gz0.w);
    vec3 g001 = vec3(gx1.x,gy1.x,gz1.x);
    vec3 g101 = vec3(gx1.y,gy1.y,gz1.y);
    vec3 g011 = vec3(gx1.z,gy1.z,gz1.z);
    vec3 g111 = vec3(gx1.w,gy1.w,gz1.w);

    vec4 norm0 = taylorInvSqrt(vec4(dot(g000, g000), dot(g010, g010), dot(g100, g100), dot(g110, g110)));
    g000 *= norm0.x;
    g010 *= norm0.y;
    g100 *= norm0.z;
    g110 *= norm0.w;
    vec4 norm1 = taylorInvSqrt(vec4(dot(g001, g001), dot(g011, g011), dot(g101, g101), dot(g111, g111)));
    g001 *= norm1.x;
    g011 *= norm1.y;
    g101 *= norm1.z;
    g111 *= norm1.w;

    float n000 = dot(g000, Pf0);
    float n100 = dot(g100, vec3(Pf1.x, Pf0.yz));
    float n010 = dot(g010, vec3(Pf0.x, Pf1.y, Pf0.z));
    float n110 = dot(g110, vec3(Pf1.xy, Pf0.z));
    float n001 = dot(g001, vec3(Pf0.xy, Pf1.z));
    float n101 = dot(g101, vec3(Pf1.x, Pf0.y, Pf1.z));
    float n011 = dot(g011, vec3(Pf0.x, Pf1.yz));
    float n111 = dot(g111, Pf1);

    vec3 fade_xyz = fade(Pf0);
    vec4 n_z = mix(vec4(n000, n100, n010, n110), vec4(n001, n101, n011, n111), fade_xyz.z);
    vec2 n_yz = mix(n_z.xy, n_z.zw, fade_xyz.y);
    float n_xyz = mix(n_yz.x, n_yz.y, fade_xyz.x); 
    return 2.2 * n_xyz;
}

float Max(vec3 v) { return max(v.x, max(v.y, v.z)); }
float Min(vec3 v) { return min(v.x, min(v.y, v.z)); }

float MinNotNull(vec3 v){
    float f = 0.5;
    float eps = 0.1;
    if(v.x > eps){
        f = v.x;
        if(v.y > eps){
            f = min(f, v.y);
        }
        if(v.z > eps){
            f = min(f, v.z);
        }
    }else if(v.y > eps){
        f = v.y;
        if(v.z > eps){
            f = min(f, v.z);
        }
    }else{
        f = v.z;
    }

    return f;
}

bool PointInOrOn(vec3 P1, vec3 P2, vec3 A, vec3 B )
{
    vec3 CP1 = cross( B - A, P1 - A );
    vec3 CP2 = cross( B - A, P2 - A );
    return dot( CP1, CP2 ) >= 0;
}

bool PointInTriangle( vec3 px, vec3 p0, vec3 p1, vec3 p2 )
{
    return 
        PointInOrOn(px, p0, p1, p2) &&
        PointInOrOn(px, p1, p2, p0) &&
        PointInOrOn(px, p2, p0, p1);
}
vec3 IntersectPlane(vec3 rd, vec3 ro, vec3 p0, vec3 p1, vec3 p2)
{
    vec3 D = rd;
    vec3 N = cross(p1-p0, p2-p0);
    vec3 X = ro + D * dot(p0 - ro, N) / dot(D, N);

    return X;
}
bool IntersectTriangle(vec3 rd, vec3 ro, vec3 p0, vec3 p1, vec3 p2)
{
    vec3 X = IntersectPlane(ro, rd, p0, p1, p2);
    return PointInTriangle(X, p0, p1, p2);
}

bool IsPointInBoxEdge(vec3 relativeHitPos, float boxSize, float threshold){
    
        vec3 percHitPos = abs(relativeHitPos) / (boxSize / 2);

        percHitPos = abs(percHitPos);
        if(max(percHitPos.x, percHitPos.y) + max(percHitPos.z, min(percHitPos.x, percHitPos.y)) > threshold * 2)
            return true;

        return false;
}
bool IsPointInBoxEdge(vec3 hitPos, vec3 boxPos, float boxSize, float threshold){
    
        return IsPointInBoxEdge(boxPos - hitPos, boxSize, threshold);
}

float IsPointInBoxEdgeThreshold(vec3 relativeHitPos, float boxSize){
    vec3 percHitPos = abs(relativeHitPos) / (boxSize / 2);
    return abs(max(percHitPos.x, percHitPos.y) + max(percHitPos.z, min(percHitPos.x, percHitPos.y)) - 1);
}
float IsPointInBoxEdgeThreshold(vec3 hitPos, vec3 boxPos, float boxSize){
        return IsPointInBoxEdgeThreshold(boxPos - hitPos, boxSize);
}

int MinIndex( float x, float y, float z )
{
   return int((y<z)&&(y<x)) + ((int((z<y)&&(z<x))*2));
}
int MinIndex( vec3 vec )
{
   return MinIndex(vec.x, vec.y, vec.z);
}

int MaxIndex( float x, float y, float z )
{
   return int((y>z)&&(y>x)) + ((int((z>y)&&(z>x))*2));
}
int MaxIndex( vec3 vec )
{
   return MaxIndex(vec.x, vec.y, vec.z);
}

vec3 RayPlaneIntersection(vec3 ro, vec3 rd, vec3 planePoint, vec3 planeNormal){
    vec3 difference = planePoint - ro;
    float product_1 = dot(difference, planeNormal);
    float product_2 = dot(rd, planeNormal);
    float distance_from_origin_to_plane = product_1 / product_2;
    vec3 intersection = ro + rd * distance_from_origin_to_plane;
    return intersection;
}
float RayPlaneIntersectionDistance(vec3 ro, vec3 rd, vec3 planePoint, vec3 planeNormal){
    vec3 difference = planePoint - ro;
    float product_1 = dot(difference, planeNormal);
    float product_2 = dot(rd, planeNormal);
    float distance_from_origin_to_plane = product_1 / product_2;
    return distance_from_origin_to_plane;
}

float RayBoxIntersection2(vec3 ro, vec3 rd, vec3 center, float halfSize){
    
    vec3 rdx = normalize(vec3(rd.x, 0, 0));
    vec3 rdy = normalize(vec3(0, rd.y, 0));
    vec3 rdz = normalize(vec3(0, 0, rd.z));

    float dx = RayPlaneIntersectionDistance(ro, rd, center + (rdx * halfSize), rdx * -1);
    float dy = RayPlaneIntersectionDistance(ro, rd, center + (rdy * halfSize), rdy * -1);
    float dz = RayPlaneIntersectionDistance(ro, rd, center + (rdz * halfSize), rdz * -1);

    return min(dx, min(dy, dz));

    // return 0;
}

float ddaRayBox(vec3 ro, vec3 rd, vec3 boxCenter, float boxHalfSize){
    vec3 boxHalfSize3 = vec3(boxHalfSize);

    //place RO on negative side of box
    
    ro = abs(ro - boxCenter - boxHalfSize3);
    // ro = ro * -1;


    vec3 roDistance = (ro);

    // if(rd.x < 0) roDistance.x = abs(ro.x - (boxHalfSize * 2));
    // if(rd.y < 0) roDistance.y = abs(ro.y - (boxHalfSize * 2));
    // if(rd.z < 0) roDistance.z = abs(ro.z - (boxHalfSize * 2));

    if(bitInt(floatBitsToInt(rd.x), 31)) roDistance.x = abs(ro.x - (boxHalfSize * 2));
    if(bitInt(floatBitsToInt(rd.y), 31)) roDistance.y = abs(ro.y - (boxHalfSize * 2));
    if(bitInt(floatBitsToInt(rd.z), 31)) roDistance.z = abs(ro.z - (boxHalfSize * 2));

    boxCenter = -boxHalfSize3;

    roDistance = abs(roDistance);

    rd = abs(rd);

    vec3 steps = roDistance / rd;
    vec3 outPoint = ro + (rd * min(steps.x, min(steps.y, steps.z)));
    // vec3 outPoint = ro + (rd * steps[MinIndex(steps)]);


    return distance(ro, outPoint);
}

float advancedRayAABB(vec3 ro, vec3 rd, vec3 lb, vec3 rt,vec3 dirfrac){
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // ro is origin of ray
    float t1 = (lb.x - ro.x)*dirfrac.x;
    float t2 = (rt.x - ro.x)*dirfrac.x;
    float t3 = (lb.y - ro.y)*dirfrac.y;
    float t4 = (rt.y - ro.y)*dirfrac.y;
    float t5 = (lb.z - ro.z)*dirfrac.z;
    float t6 = (rt.z - ro.z)*dirfrac.z;

    return min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
}

int RussianPeasant(float value, float multiplier){

    float v = multiplier;
    int i = 0;
    while(v < value)
    {
        v = v * LOD_RPM_MULTIPLIER;
        i++;
    }
    return i;
}


mat3 getCam(vec3 camF, vec3 camR, vec3 camU){
    return mat3(camR, camU, camF);
}

bool slabs(vec3 p0, vec3 p1, vec3 rayOrigin, vec3 invRaydir) {
    vec3 t0 = (p0 - rayOrigin) * invRaydir;
    vec3 t1 = (p1 - rayOrigin) * invRaydir;
    vec3 tmin = min(t0,t1), tmax = max(t0,t1);
    return Max(tmin) <= Min(tmax);
}

bool advancedIntersectAABB(vec3 ro, vec3 rd, vec3 lb, vec3 rt, out float tmin, out float tmax){
    // rd is unit direction vector of ray
    vec3 dirfrac = vec3(0);
    dirfrac.x = 1.0f / rd.x;
    dirfrac.y = 1.0f / rd.y;
    dirfrac.z = 1.0f / rd.z;
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // ro is origin of ray
    float t1 = (lb.x - ro.x)*dirfrac.x;
    float t2 = (rt.x - ro.x)*dirfrac.x;
    float t3 = (lb.y - ro.y)*dirfrac.y;
    float t4 = (rt.y - ro.y)*dirfrac.y;
    float t5 = (lb.z - ro.z)*dirfrac.z;
    float t6 = (rt.z - ro.z)*dirfrac.z;

    tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0)
    {
        // t = tmax;
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        // t = tmax;
        return false;
    }

    // t = tmin;
    return true;
}

bool advancedIntersectAABBdirfrac(vec3 ro, vec3 rd, vec3 lb, vec3 rt,vec3 dirfrac, out float tmin, out float tmax){
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // ro is origin of ray
    float t1 = (lb.x - ro.x)*dirfrac.x;
    float t2 = (rt.x - ro.x)*dirfrac.x;
    float t3 = (lb.y - ro.y)*dirfrac.y;
    float t4 = (rt.y - ro.y)*dirfrac.y;
    float t5 = (lb.z - ro.z)*dirfrac.z;
    float t6 = (rt.z - ro.z)*dirfrac.z;

    tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0)
    {
        // t = tmax;
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        // t = tmax;
        return false;
    }

    // t = tmin;
    return true;
}

void aabbIntersectionDistance(vec3 ro, vec3 rd, vec3 lb, vec3 rt,vec3 dirfrac, out float tmin, out float tmax){
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // ro is origin of ray
    float t1 = (lb.x - ro.x)*dirfrac.x;
    float t2 = (rt.x - ro.x)*dirfrac.x;
    float t3 = (lb.y - ro.y)*dirfrac.y;
    float t4 = (rt.y - ro.y)*dirfrac.y;
    float t5 = (lb.z - ro.z)*dirfrac.z;
    float t6 = (rt.z - ro.z)*dirfrac.z;

    tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
}


void aabbIntersectionOutDistance(vec3 ro, vec3 rd, vec3 lb, vec3 rt,vec3 dirfrac, out float tmin, out float tmax){
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // ro is origin of ray
    float t1 = (lb.x - ro.x)*dirfrac.x;
    float t2 = (rt.x - ro.x)*dirfrac.x;
    float t3 = (lb.y - ro.y)*dirfrac.y;
    float t4 = (rt.y - ro.y)*dirfrac.y;
    float t5 = (lb.z - ro.z)*dirfrac.z;
    float t6 = (rt.z - ro.z)*dirfrac.z;

    // tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
}

bool intersectAABB(vec3 rayOrigin, vec3 rayDir, vec3 boxMin, vec3 boxMax, out float dist = 0) {
    vec3 tMin = (boxMin - rayOrigin) / rayDir;
    vec3 tMax = (boxMax - rayOrigin) / rayDir;
    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);
    dist = tNear;
    if (tFar < 0) return false;
    return tNear < tFar;
}
float intersectAABBdist(vec3 rayOrigin, vec3 rayDir, vec3 boxMin, vec3 boxMax) {
    vec3 tMin = (boxMin - rayOrigin) / rayDir;
    vec3 tMax = (boxMax - rayOrigin) / rayDir;
    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);

    return tNear;
}
bool intersectAABBdist(vec3 rayOrigin, vec3 rayDir, vec3 boxMin, vec3 boxMax, out float tNear, out float tFar) {
    vec3 tMin = (boxMin - rayOrigin) / rayDir;
    vec3 tMax = (boxMax - rayOrigin) / rayDir;
    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);
    tNear = max(max(t1.x, t1.y), t1.z);
    tFar = min(min(t2.x, t2.y), t2.z);

    if (tFar < 0) return false;
    return tNear < tFar;

    // return tNear;
}

bool raySphereIntersect(vec3 r0, vec3 rd, vec3 s0, float sr) {
    float a = dot(rd, rd);
    vec3 s0_r0 = r0 - s0;
    float b = 2.0 * dot(rd, s0_r0);
    float c = dot(s0_r0, s0_r0) - (sr * sr);
    return (b*b - 4.0*a*c > 0.0);
}
bool raySphereIntersectDist(vec3 r0, vec3 rd, vec3 s0, float sr, out float dist) {
    float a = dot(rd, rd);
    vec3 s0_r0 = r0 - s0;
    float b = 2.0 * dot(rd, s0_r0);
    float c = dot(s0_r0, s0_r0) - (sr * sr);
    if (b*b - 4.0*a*c < 0.0) {
        dist = 0;
        return false;
    }else{
        dist = (-b - sqrt((b*b) - 4.0*a*c))/(2.0*a);
        return true;
    }
}

int getNodeOffset(int[11] steps, int initialOffset){
    int offset = 5;
    
    for(int i = 0;i < 10;i++){
        if(steps[i] == -1){
            return offset;
        }

        for(int j = 0;j < steps[i];j++){
            int stateBytes = floatBitsToInt(TexelFetch1(offset + 1));
            int voxColor = floatBitsToInt(TexelFetch1(4));
            if(bitInt(stateBytes, OctDivided)){
                offset += voxColor + 1;
            }else{
                offset += 2;
            }
        }
    }
    return offset;
}


void getVoxelAABB(vec3 center, vec3 size, out vec3 minCorner, out vec3 maxCorner){
    minCorner = center - (size / 2);
    maxCorner = center + (size / 2);
    return;
}

int IndicesToIndex(ivec3 indices, int ssbo = 0){
    return ssbo * VOXELS_PER_CHUNK_EDGE * VOXELS_PER_CHUNK_EDGE * VOXELS_PER_CHUNK_EDGE
        + indices.z * VOXELS_PER_CHUNK_EDGE * VOXELS_PER_CHUNK_EDGE
        + indices.y * VOXELS_PER_CHUNK_EDGE
        + indices.x;
}

ivec3 IndexToIndices(int index){
    ivec3 ret = ivec3(0);
    ret.x = index % VOXELS_PER_CHUNK_EDGE;
    index = (index-ret.x)/VOXELS_PER_CHUNK_EDGE;

    ret.y = index % VOXELS_PER_CHUNK_EDGE;
    index = (index-ret.y)/VOXELS_PER_CHUNK_EDGE;

    ret.z = index % VOXELS_PER_CHUNK_EDGE;

    return ret;
}

void ClearVoxelArray(int ssbo = 0){
    // for(int i = 0;i < VOXELS_PER_CHUNK / 2;i++){
    //     DefVoxelArray[i] = -1;
    // }

    return;

    for(int x = 0;x < VOXELS_PER_CHUNK_EDGE;x++)
        for(int y = 0;y < VOXELS_PER_CHUNK_EDGE;y++)
            for(int z = 0;z < VOXELS_PER_CHUNK_EDGE;z++){
                DefVoxelArray[IndicesToIndex(ivec3(x,y,z))] = -2;
            }
}

// ivec3 RelativePositionToArrayIndices(vec3 pos){
//     //transform corner positions to [0,16]
//     pos += vec3(HALF_CHUNK_SIZE);
//     //transform corner positions to [0,1]
//     pos /= vec3(CHUNK_SIZE);
//     //convert relative position in chunk to indices in array
//     ivec3 ind = clamp(ivec3(vec3(255) * pos), ivec3(0), ivec3(255));
    
//     if (ind.x >= CHUNK_SIZE) ind.x = CHUNK_SIZE - 1;
//     if (ind.y >= CHUNK_SIZE) ind.y = CHUNK_SIZE - 1;
//     if (ind.z >= CHUNK_SIZE) ind.z = CHUNK_SIZE - 1;

//     if (ind.x < 0) ind.x = 0;
//     if (ind.y < 0) ind.y = 0;
//     if (ind.z < 0) ind.z = 0;

//     return ind;
// }


Voxel GetVoxel(int index, samplerBuffer tbo = u_tbo_tex){
    //col
    //state
    //pos
    int leafsUnder = floatBitsToInt(TexelFetch1(CHUNK_INFO_START_POINTER + (FLOATS_PER_VOXEL * index), tbo));
    int state = floatBitsToInt(TexelFetch1(CHUNK_INFO_START_POINTER + (FLOATS_PER_VOXEL * index) + 1, tbo));
    vec4 color = colorFromBytes(floatBitsToInt(TexelFetch1(CHUNK_INFO_START_POINTER + (FLOATS_PER_VOXEL * index) + 2, tbo)));
    vec3 position = vec3(0);
    int leafLevel = 0;
    return Voxel(color, state, leafsUnder, position, leafLevel);
}

Voxel GetVoxelByPointer(int pointer, samplerBuffer tbo = u_tbo_tex){
    //col
    //state
    //pos
    int leafsUnder = floatBitsToInt(TexelFetch1(pointer, tbo));
    int state = floatBitsToInt(TexelFetch1(pointer + 1, tbo));
    vec4 color = colorFromBytes(floatBitsToInt(TexelFetch1(pointer + 2, tbo)));
    vec3 position = vec3(0);
    int leafLevel = 0;
    return Voxel(color, state, leafsUnder, position, leafLevel);
}

Ray2VoxelHit VoxelWithPointInside(
    vec3 point,
    vec3 chunkCenter,
    vec3 ro,
    int min_layer = MAX_VOXEL_DIVIDE_COUNT,
    out int iterations,
    out int iterations_skipped,
    out vec3[OCTREE_MAX_LAYER] prevNodePosition,
    out int[OCTREE_MAX_LAYER] prevNodeIndex,
    int prevLayer = -1){



    int deepest_layer = MAX_VOXEL_DIVIDE_COUNT + 1;
    int nodePointer = CHUNK_INFO_START_POINTER;
    vec3 nodeCenter = chunkCenter;
    

    int startIndex = 0;

    iterations = 0;
    iterations_skipped = -1;

    #ifdef DEV_BACKTRACKING

    if(DEV_BACKTRACKING_MODE % 2 == 1){

        if(prevLayer > 2){
            for(int i = prevLayer - 1;i > -1;i--){

                float sizeMod = pow(2, i);
                float halfSize = CHUNK_SIZE / sizeMod / 4.0;
                // float halfSize = OCTREE_VOXEL_SIZES[i + 1] / 4.0;

                startIndex = i;
                nodePointer = prevNodeIndex[i];
                nodeCenter = prevNodePosition[i];

                
                // iterations++;
                // if(IsPointInside(point, nodeCenter - vec3(halfSize), nodeCenter + vec3(halfSize))){
                if(IsPointInside_2(point, nodeCenter, vec3(halfSize * 2))){
                    iterations_skipped = i;
                    break;
                }
                if(startIndex < 0){
                    iterations_skipped = i;
                    startIndex = 0;
                    nodePointer = CHUNK_INFO_START_POINTER;
                    nodeCenter = chunkCenter;
                    break;
                }
            }
        }
    }

    #endif

    for(int i = startIndex; i < deepest_layer; i++){
        float sizeMod = pow(2, i);
        float halfSize = CHUNK_SIZE / sizeMod / 4;
        // float halfSize = OCTREE_VOXEL_SIZES[i + 1] / 4.0;
        
        // int m_layer = abs(int(floor(distance(nodeCenter, ro) / LOD_DISTANCE_PER_LAYER)));
        // m_layer = max(LOD_MIN_LAYER, deepest_layer - m_layer);


        prevNodeIndex[i] = nodePointer;
        prevNodePosition[i] = nodeCenter;

        iterations++;
        if(floatBitsToInt(TexelFetch1(nodePointer)) == 1 || i == min_layer){
            return Ray2VoxelHit(nodeCenter, halfSize * 2, i, nodePointer);
        }
        

        int childNodeIndex = 0;
        vec3 relative_point = normalize(point - nodeCenter);
        
        if(relative_point.y < 0) childNodeIndex += 4;   // if -
        if(relative_point.z > 0){
             childNodeIndex += 2;   // if +
        }else{
             relative_point.x *= -1;
        }
        if(relative_point.x < 0) childNodeIndex += 1;   // if -
        
        #ifdef DEV_CHILDNODE_SKIPPING

            nodePointer += (1 + floatBitsToInt(TexelFetch1(nodePointer + 4 + childNodeIndex))) * FLOATS_PER_VOXEL;

        #else

            nodePointer += FLOATS_PER_VOXEL;
            for(int j = 0;j < childNodeIndex;j++){
                nodePointer += (floatBitsToInt(TexelFetch1(nodePointer))) * FLOATS_PER_VOXEL;
                iterations++;
            }

        #endif
        
        
        nodeCenter = nodeCenter + (OCTREE_DIAGONAL_DIRECTIONS[childNodeIndex] * halfSize);
    }

    return Ray2VoxelHit(chunkCenter, 0, 0, 8);
}


bool rayChunk(vec3 ro, vec3 rd, int tbo, out float dist){
    vec3 chunkCenter = vec3(
        (tbo * CHUNK_SIZE),
        TexelFetch1(1),
        TexelFetch1(2)
    );

    vec3 chunkPos[9] = vec3[9](
        vec3(0,0,0),
        vec3(0,0,1),
        vec3(0,0,-1),
        vec3(1,0,0),
        vec3(1,0,1),
        vec3(1,0,-1),
        vec3(-1,0,0),
        vec3(-1,0,1),
        vec3(-1,0,-1)
    );
    chunkCenter = chunkPos[tbo] * CHUNK_SIZE;
    
    float distOut = 0;
    
    vec3 dirfrac = vec3(0);
    dirfrac.x = 1.0f / rd.x;
    dirfrac.y = 1.0f / rd.y;
    dirfrac.z = 1.0f / rd.z;

    bool intersect = advancedIntersectAABBdirfrac(ro, rd, chunkCenter - vec3(HALF_CHUNK_SIZE - EPSILON), chunkCenter + vec3(HALF_CHUNK_SIZE - EPSILON), dirfrac, dist, distOut);
    
    return intersect;
}

VoxelHit renderChunk_1(vec3 ro, vec3 rd, int tbo, float MAX_DISTANCE = MAX_RENDER_DISTANCE){
    int leafsCount = floatBitsToInt(TexelFetch1(6));
    vec3 chunkCenter = vec3(
        (tbo * CHUNK_SIZE),
        TexelFetch1(1),
        TexelFetch1(2)
    );

    vec3 chunkPos[9] = vec3[9](
        vec3(0,0,0),
        vec3(0,0,1),
        vec3(0,0,-1),
        vec3(1,0,0),
        vec3(1,0,1),
        vec3(1,0,-1),
        vec3(-1,0,0),
        vec3(-1,0,1),
        vec3(-1,0,-1)
    );
    chunkCenter = chunkPos[tbo] * CHUNK_SIZE;

    Voxel voxel = GetVoxel(0);

    float dist = 0;
    float distOut = 0;


    Ray2VoxelHit pointhit = Ray2VoxelHit(vec3(0), 0, -1, -1);
    int iterations = 0;
    int _iterations_skipped;
    int vwpi_iterations = 0;
    int raybox_iterations = 0;
    int _iter = 0;
    int _iter2 = 0;

    vec3 dirfrac = vec3(0);
    dirfrac.x = 1.0f / rd.x;
    dirfrac.y = 1.0f / rd.y;
    dirfrac.z = 1.0f / rd.z;
    
    bool intersect = advancedIntersectAABBdirfrac(ro, rd, chunkCenter - vec3(HALF_CHUNK_SIZE - EPSILON), chunkCenter + vec3(HALF_CHUNK_SIZE - EPSILON), dirfrac, dist, distOut);
    raybox_iterations++;
    VoxelHit vh = VoxelHit(vec3(0), vec3(0), vec3(0), MAX_DISTANCE, MAX_DISTANCE, CHUNK_SIZE, vec4(0), false, 0);

    if(intersect){
        vec3 prevNodePosition[OCTREE_MAX_LAYER] = vec3[OCTREE_MAX_LAYER](vec3(0),vec3(0),vec3(0),vec3(0),vec3(0),vec3(0),vec3(0),vec3(0),vec3(0),vec3(0),vec3(0));
        int prevNodeIndex[OCTREE_MAX_LAYER] = int[OCTREE_MAX_LAYER](-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1);

        bool isInside = intersect;

        vec3 pointPos = ro + (rd * (dist + EPSILON));

        if(IsPointInside_2(ro, chunkCenter, vec3(HALF_CHUNK_SIZE))){

            #ifdef RO_INSIDE_CHUNK_DOUBLE_CHECK
            

            pointhit = VoxelWithPointInside(ro, chunkCenter, ro, MAX_VOXEL_DIVIDE_COUNT, _iter, _iter2, prevNodePosition, prevNodeIndex, -1);
            vwpi_iterations += _iter;
            _iterations_skipped += _iter2;
            voxel = GetVoxelByPointer(pointhit.voxelIndex);
            
            vwpi_iterations+=3;

            vh.crossed = bitInt(voxel.data, OctFillState) || bitInt(voxel.data, OctDivided);
            if(vh.crossed){
                aabbIntersectionDistance(ro - (rd * CHUNK_SIZE * 2), rd, pointhit.voxelCenter - vec3(pointhit.voxelHalfSize), pointhit.voxelCenter + vec3(pointhit.voxelHalfSize), dirfrac, dist, distOut);
                raybox_iterations++;
                vec3 rayHitPosition = ro - (rd * CHUNK_SIZE * 2) + (rd * dist);
                vh.pos = pointhit.voxelCenter;
                vh.hit = ro + (rd * dist);
                vh.dist = dist;
                vh.distOut = distOut;
                vh.voxelSize = pointhit.voxelHalfSize * 2;
                vh.crossed = bitInt(voxel.data, OctFillState) || bitInt(voxel.data, OctDivided);
                vh.color = voxel.color;
                vh.state = voxel.data;
                vh.normal = BoxNormal(normalize(vh.hit - vh.pos));
                isInside = false;
            }
            #endif
            vh.color = voxel.color;
            vh.state = voxel.data;
            pointPos = ro;
        }

        while(isInside == true && iterations < 512){
            iterations++;
            isInside = true;

            int m_layer = RussianPeasant(distance(pointPos, ro), LOD_DISTANCE_PER_LAYER);
            m_layer = max((-m_layer) + MAX_VOXEL_DIVIDE_COUNT, LOD_MIN_LAYER);

            pointhit = VoxelWithPointInside(pointPos, chunkCenter, ro, m_layer, _iter, _iter2, prevNodePosition, prevNodeIndex, pointhit.voxelLayer);
            vwpi_iterations += _iter;
            _iterations_skipped += _iter2;
            voxel = GetVoxelByPointer(pointhit.voxelIndex);
            vwpi_iterations+=3;

            vh.crossed = bitInt(voxel.data, OctFillState) || bitInt(voxel.data, OctDivided);
            vh.color = voxel.color;
            vh.state = voxel.data;

            

            if(vh.crossed == false){

                // // // // newset alg
                // distOut = advancedRayAABB(ro, rd, pointhit.voxelCenter - vec3(pointhit.voxelHalfSize + EPSILON), pointhit.voxelCenter + vec3(pointhit.voxelHalfSize + EPSILON), dirfrac);
                // raybox_iterations++;
                // vec3 lastPos = pointPos;
                // pointPos = ro + (rd * (distOut));


                // // // // new alg
                // distOut = ddaRayBox(pointPos, rd, pointhit.voxelCenter, pointhit.voxelHalfSize + EPSILON);
                // raybox_iterations++;
                // vec3 lastPos = pointPos;
                // pointPos = pointPos + (rd * (distOut + EPSILON));


                // // // old alg
                aabbIntersectionDistance(ro, rd, pointhit.voxelCenter - vec3(pointhit.voxelHalfSize + EPSILON), pointhit.voxelCenter + vec3(pointhit.voxelHalfSize + EPSILON), dirfrac, dist, distOut);
                raybox_iterations++;
                vec3 lastPos = pointPos;
                pointPos = ro + (rd * (distOut));


                if(distance(lastPos, pointPos) < EPSILON) pointPos = lastPos + (rd * EPSILON_M);

                // if(distOut > MAX_DISTANCE) break;
            }else{
                {
                    aabbIntersectionDistance(ro, rd, pointhit.voxelCenter - vec3(pointhit.voxelHalfSize + EPSILON), pointhit.voxelCenter + vec3(pointhit.voxelHalfSize + EPSILON), dirfrac, dist, distOut);
                    raybox_iterations++;
                    vh.pos = pointhit.voxelCenter;
                    vh.hit = ro + (rd * dist);
                    vh.dist = dist;
                    vh.distOut = distOut;
                    vh.voxelSize = pointhit.voxelHalfSize * 2;
                    vh.crossed = bitInt(voxel.data, OctFillState) || bitInt(voxel.data, OctDivided);
                    vh.color = voxel.color;
                    vh.state = voxel.data;
                    vh.normal = BoxNormal(normalize(vh.hit - vh.pos));
                    break;
                }
            }

            isInside = IsPointInside_2(pointPos, chunkCenter, vec3(HALF_CHUNK_SIZE));
        }

        
        if(distance(pointPos, ro) > MAX_DISTANCE){
            vh.crossed = false;
        }
        
        #ifdef VISUALIZE_ITERATIONS
        switch(VISUALIZE_ITERATIONS_MODE % VISUALIZE_ITERATIONS){
            case 1:{
                vh.color = vec4(vec3((iterations / VISUALIZE_ITERATIONS_MULTIPLIER)), 1);
                break;
            }
            case 2:{
                vh.color = vec4((vwpi_iterations / VISUALIZE_ITERATIONS_MULTIPLIER_2),0,0, 1);
                break;
            }
            case 3:{
                vh.color = vec4(0,(raybox_iterations / VISUALIZE_ITERATIONS_MULTIPLIER_3),0, 1);
                break;
            }
            case 4:{
                vh.color = vec4(0,0, (_iterations_skipped / VISUALIZE_ITERATIONS_MULTIPLIER_4), 1);
                break;
            }
        }
        #endif
    }

    // vec3 chunkRotation = vec3(0, u_time, 0);
    
    // rotate(vh.pos, chunkRotation);
    // rotate(vh.hit, chunkRotation);
    // rotate(vh.normal, chunkRotation);

    return vh;
}


VoxelHit renderChunk(vec3 ro, vec3 rd, int tbo, float MAX_DISTANCE = MAX_RENDER_DISTANCE){
        return renderChunk_1(ro,rd,tbo, MAX_DISTANCE);
}

VoxelHit RenderChunks(vec3 ro, vec3 rd, float MAX_DISTANCE = MAX_RENDER_DISTANCE){

    vec4 col = vec4(0);
    // VoxelHit vh = renderChunk(ro, rd, 0, MAX_DISTANCE);
    // col += vh.color;
    VoxelHit vh = VoxelHit(vec3(0), vec3(0), vec3(0), MAX_DISTANCE, MAX_DISTANCE, CHUNK_SIZE, vec4(0), false, 0);


    for(int i = 0;i < DEV_CHUNKS_COUNT;i++){

        float dist = 0;
        if(rayChunk(ro,rd,i,dist)){
            if(dist > vh.dist) continue;
        }else{
            continue;
        }
    
        VoxelHit h = renderChunk(ro, rd, i, MAX_DISTANCE);
        col += h.color;
        if(vh.crossed == false && h.crossed == true){
            vh = h;
        }
        else if(h.dist < vh.dist){
            vh = h;
        }
    }

    #ifdef VISUALIZE_ITERATIONS
    
        if(VISUALIZE_ITERATIONS_MODE % VISUALIZE_ITERATIONS != 0)
            vh.color = col;
    #endif

    return vh;
}


vec4 render(vec2 uv){
    vec4 col = vec4(0,0,0,0);
    vec3 ro = u_camera_position;
    float ratio = u_resolution.x / u_resolution.y;
    vec3 rd = getCam(u_camera_forward, u_camera_right, u_camera_up) * normalize((vec3(uv, ratio * FOV * u_mouse_wheel)));

    vec4 color = vec4(0);
    vec4 temp = vec4(0);

    VoxelHit vh = RenderChunks(ro, rd);

    
    // if(vh.crossed)
    //     return vec4(0,1,0, 1);
    // else
    //     return vec4(1,0,0,1);


    #ifdef DISTANCE_FIELD_DEBUG
    float bwDistance = 1.0 / MAX_RENDER_DISTANCE * vh.dist;

    if(vh.crossed)
        return vec4(vec3(bwDistance), 1);
    else
        return vec4(vec3(0),0);
    #endif

    
    #ifdef VISUALIZE_ITERATIONS
    
        if(VISUALIZE_ITERATIONS_MODE % VISUALIZE_ITERATIONS != 0)
            return vh.color;
    #endif

    
    if(vh.crossed){
        

        
        // float lightDot = (dot(vh.normal, globalLightDirection) * -1 + 1) / 4;
        float lightDot = dot(vh.normal, globalLightDirection) * -1;

        color = vh.color;
        
        #ifdef LIGHT_DOT_SHADOWS
        if(lightDot > 0 && abs(lightDot) >= 0){
            // lited side
            vec3 invSunDirection = globalLightDirection * -1;
            vec3 startPos = vh.hit + (vh.normal * EPSILON);
            float insideDistMultiplier = 0;
            bool crossed = false;

            
            // // // // // Soft shadows future implementation

            // int itter = -1;
            // VoxelHit _vh;
            // vec4 _col = vec4(1);
            // for(int i = 0;i < 3;i++){
            //     VoxelHit sunlightRay = RenderChunks(startPos, invSunDirection);
            //     // startPos += (invSunDirection * (abs(sunlightRay.distOut - sunlightRay.dist) + 1));
            //     if(sunlightRay.crossed){
            //         _vh = sunlightRay;
            //         float percent = (abs(_vh.distOut - _vh.dist) / (sunlightRay.voxelSize / 2));
            //         insideDistMultiplier += percent;
            //         if(i == 2){
            //             _col = _vh.color;
            //             insideDistMultiplier = percent;
            //         }
            //         startPos = _vh.hit + (invSunDirection * (abs(sunlightRay.distOut - sunlightRay.dist) + 0.1));
            //         // insideDistMultiplier += abs(sunlightRay.distOut - sunlightRay.dist) / (sunlightRay.voxelSize * (0.35 * (i + 1)));
            //         // insideDistMultiplier += abs(sunlightRay.distOut - sunlightRay.dist) / (OCTREE_VOXEL_SIZES[10]);
            //         // insideDistMultiplier += percent;
            //         // insideDistMultiplier += percent > 0.5 ? 1 : percent;
            //         // insideDistMultiplier += 0.5;
            //         crossed = true;
            //         itter++;
            //     }else{
            //         break;
            //     }
            // }

            bool globalCrossed = false;
            #ifdef GLOBAL_ILLUMINATION

            
            #ifdef PER_VOXEL_ILLUMINATION
            startPos = (startPos - mod(startPos, PER_VOXEL_ILLUMINATION)) + (vh.normal * PER_VOXEL_ILLUMINATION);
            #endif

            bool AONear = false;


            VoxelHit sunlightRay = RenderChunks(startPos, invSunDirection);
            if(sunlightRay.crossed){
                // insideDistMultiplier += (sunlightRay.distOut - sunlightRay.dist) / (sunlightRay.voxelSize / 1);
                if(sunlightRay.dist < OCTREE_VOXEL_SIZES[7] / 2) AONear = true;
                insideDistMultiplier += abs(sunlightRay.distOut - sunlightRay.dist) / (OCTREE_VOXEL_SIZES[9]);
                globalCrossed = true;
            }

            #endif


            if(globalCrossed){
                // lited side, but in shadow

                float shadowMult = ShadowMaxCap;

                #ifdef CAMERA_ILLUMINATION
                float _dist = distance(ro, startPos);
                if(_dist < CAMERA_ILLUMINATION){
                    shadowMult = ShadowMaxCap * (1.0 / CAMERA_ILLUMINATION * _dist);
                }
                #endif
                
                color = ColorBlend(vh.color, vec4(0,0,0,1), 1, shadowMult);
                
                // // // // // Soft shadows future implementation

                // insideDistMultiplier = clamp(insideDistMultiplier, 0.0, 1);
                // color = ColorBlend(vh.color, vec4(0,0,0,1), 1, abs(lightDot) * (ShadowDotMultiplier * insideDistMultiplier));
                // color = vec4(vec3(insideDistMultiplier),1);
                // color = _col;
                // color = _vh.color;
                // color = COLOR_PALETTE[itter];
                // color = ColorBlend(vh.color, vec4(0,0,0,1), 1, insideDistMultiplier * 99);
                // color = COLOR_PALETTE[1];
                // if(sunlightRay.dist > 10) color = vec4(0,0,1,1);
            }else{
                // lited side without shadows
                color = ColorBlend(vh.color, globalLightColor, 1, tan(lightDot) / LightDotMultiplier);
                
            }
            // color = ColorBlend(vh.color, globalLightColor, 1, tan(lightDot) / LightDotMultiplier);
        }else{
            // NOT lited side      
            float shadowMult = ShadowMaxCap;
            
            #ifdef CAMERA_ILLUMINATION
            float _dist = vh.dist;
            if(_dist < CAMERA_ILLUMINATION){
                shadowMult = ShadowMaxCap * (1.0 / CAMERA_ILLUMINATION * _dist);
            }
            #endif
            
            color = ColorBlend(vh.color, vec4(0,0,0,1), 1, clamp(abs(lightDot) * ShadowDotMultiplier,0, shadowMult));
        }
        #endif

        #ifdef AMBIENT_OCCLUSION

            if(IsPointInBoxEdgeThreshold(vh.hit, vh.pos, vh.voxelSize) > 1.0 - (1.0 / vh.voxelSize * MIN_VOXEL_SIZE) ){
                
                vec3 sPoint = vh.hit - (vh.normal, EPSILON);
                sPoint = (sPoint - mod(sPoint, MIN_VOXEL_SIZE));

                vec3 aoDir = normalize(vh.hit - vh.pos);
                int ind = MinIndex(abs(aoDir));
                aoDir = vec3(
                    1.0 * sign(aoDir.x),
                    1.0 * sign(aoDir.y),
                    1.0 * sign(aoDir.z)
                );
                
                // aoDir[ind] = 0;
                sPoint += aoDir * MIN_VOXEL_SIZE;
                // float d = aoDir[ind];
                // aoDir = vec3(0);
                // aoDir[ind] = d;


                // if(IsPointInBoxEdge(vh.hit, vh.pos, vh.voxelSize, 0.9)){
                    // color = vec4(0,0,0,1);
                    // color[ind] = 1;
                // }


                
                
                VoxelHit aoRay = RenderChunks(sPoint, vh.normal, MIN_VOXEL_SIZE * 3);
                if(aoRay.crossed){
                    // insideDistMultiplier += (sunlightRay.distOut - sunlightRay.dist) / (sunlightRay.voxelSize / 1);
                    // color = ColorBlend(vh.color, vec4(0,0,0,0), 1, 100);
                    color = vec4(0,0,0,1);
                    // color = aoRay.color;
                    // color[ind] = 1;

                }
                // aoDir.x = 0;
                // aoDir.y = 0;
                // aoDir.z = 0;
                // vec3 d = vec3(0);
                // d[ind] = 1;
                // color = vec4(abs(aoDir),1);
                    // color[ind] = 1;
            }

        #endif

        
        #ifdef VOXEL_NOISE
            float noise = noise3((vh.hit - mod(vh.hit, VOXEL_NOISE_SIZE)) * CHUNK_SIZE * vec3(10,100,1000));
            color = ColorBlend(color, vec4(vec3(noise), 1), 1, VOXEL_NOISE_MULTIPLIER);
        #endif

        #ifdef VOXEL_NORMAL_DEBUG
            return vec4(abs(vh.normal),1);
        #endif

        #ifdef DRAW_VOXEL_BORDERS
            if(IsPointInBoxEdge(vh.hit, vh.pos, vh.voxelSize, 0.99)){
                vec4 bordColor = invertColor(vh.color);
                color = vec4(1);
            }

            // vh.color = bordColor
        #endif
    }else{
            color = backgroundColor;
        #ifdef SUN_BLOOM

            float d = dot(rd * -1, normalize(globalLightDirection));
            if(d > 1 - SUN_BLOOM){
                color = globalLightColor;
            }

        #endif
        
    }
    

    #ifdef WIRECUBE_DEBUG

    vec3 pos = ro + (u_camera_forward * WIRECUBE_DISTANCE);
    pos = pos - mod(pos, vec3(0.25));
    float dist = 0;
    float distOut = 0;

    if(advancedIntersectAABB(ro, rd, pos - vec3(0.5), pos + vec3(0.5), dist, distOut)){
        vec3 hitpos = ro + (rd * dist);
        vec3 relhitpos = abs(hitpos - pos) * 2;
        
        vec3 hitposout = ro + (rd * distOut);
        vec3 relhitposout = abs(hitposout - pos) * 2;

        float f = 0.9;
        if(
            (relhitpos.x > f && relhitpos.y > f) ||
            (relhitpos.x > f && relhitpos.z > f) ||
            (relhitpos.z > f && relhitpos.y > f)
        ){
            if(dist > vh.dist)
                color = vec4(0,1,0,0.5);
            else
                color = vec4(0,1,0,1);
        }else if(
            (relhitposout.x > f && relhitposout.y > f) ||
            (relhitposout.x > f && relhitposout.z > f) ||
            (relhitposout.z > f && relhitposout.y > f)
        ){
            if(distOut > vh.dist)
                color = vec4(0,1,0,0.5);
            else
                color = vec4(0,1,0,1);
        }
    }

    #endif

    // if(IntersectTriangle(ro,rd, vec3(0,-1000,0), vec3(0,1000, 0), vec3(1,1000, 0))){
    //     color = vec4(0,0,1,1);
    // }

    return color;
}


vec2 getUV(vec2 offset, float div = 1){
    return (2.0 * (gl_FragCoord.xy + offset) - (u_resolution.xy / div)) / (u_resolution.y / div);
}

vec4 renderAAx4(){
    vec4 e = vec4(0.125, -0.125, 0.375, -0.375);
    vec4 colAA = render(getUV(e.xz)) + render(getUV(e.yw)) + render(getUV(e.wx)) + render(getUV(e.zy));
    return colAA / 4.0;
}


vec4 renderAAx2() {
    float bxy = int(gl_FragCoord.x + gl_FragCoord.y) & 1;
    float nbxy = 1. - bxy;
    vec4 colAA = (render(getUV(vec2(0.33 * nbxy, 0.))) + render(getUV(vec2(0.33 * bxy, 0.66))));
    return colAA / 2.0;
}


vec4 renderAAx3() {
    float bxy = int(gl_FragCoord.x + gl_FragCoord.y) & 1;
    float nbxy = 1. - bxy;
    vec4 colAA = (render(getUV(vec2(0.66 * nbxy, 0.))) +
                  render(getUV(vec2(0.66 * bxy, 0.66))) +
                  render(getUV(vec2(0.33, 0.33))));
    return colAA / 3.0;
}

void main()
{

    float ratio = u_resolution.x / u_resolution.y;
    vec2 resol = vec2(u_resolution.x / 4, u_resolution.x / 4 / ratio);
    vec2 uv = (0.5 * gl_FragCoord.xy - resol.xy) / resol.y;

    // fragColor = vec4(uv.x, uv.y, 0, 1);
    // if(gl_FragCoord.x < 100 && gl_FragCoord.y < 100) fragColor = vec4(0);
    // else fragColor = vec4(1);
    // // fragColor = vec4(resol.x, resol.y, 0, 1);
    // return;

    vec4 color = vec4(0);
    if(NoiseRender == 1){
        if(rand((uv - mod(uv, 0.001)) * u_time * 1) > 0.5){
        // if(rand(uv * u_time) > NoiseRenderThreshold){
            switch(u_AA_type){
                case 0: color = render(uv); break;
                case 1: color = renderAAx2(); break;
                case 2: color = renderAAx3(); break;
                case 3: color = renderAAx4(); break;
            }
        }
    }else{
        switch(u_AA_type){
            case 0: color = render(uv); break;
            case 1: color = renderAAx2(); break;
            case 2: color = renderAAx3(); break;
            case 3: color = renderAAx4(); break;
        }
    }
    
                // color = render(uv);
    // color = render(uv);

    

    if(uv.x > -0.01 && uv.x < 0.01 && uv.y > -0.01 && uv.y < 0.01){
        color = vec4(1);
        bool b = false;
        vec3 v = vec3(TexelFetch4(0));
        b = v.y == 1;
        if(b){
            color = vec4(0,1,0,1);
        }else{
            color = vec4(1,0,0,1);
        }
    }
    

    // if(PointAABB(vec3(0.9999999), vec3(-1), vec3(1))){
    //     color = vec4(1);
    // }

    
    //gamma
    vec4 col = color;
    
    // col = pow(color, vec4(0.4545,0.4545,0.4545,1));
    col = pow(color, vec4(vec3(0.75),1));
    // col = pow(color, vec4(0.4));

    col.w = color.w;
    fragColor = col;


    // fragColor = color;
}

