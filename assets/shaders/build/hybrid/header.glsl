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

