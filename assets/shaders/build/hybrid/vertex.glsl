
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