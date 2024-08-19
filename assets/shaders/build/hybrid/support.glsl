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