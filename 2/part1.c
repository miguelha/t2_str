#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>

#define BUF_SIZE 64

typedef struct{
    float* x;
    float* y;
    float* z;

    int numPts;
} pointCloud;

typedef struct{
    float minX;
    float maxX;
    float avgX;
    float stdX;
    float minY;
    float maxY;
    float avgY;
    float stdY;
    float minZ;
    float maxZ;
    float avgZ;
    float stdZ;
} statMetrics;

// AUXILIARY FUNCTIONS

long long int calculateElapsedTime(struct timespec init, struct timespec end){
    long long int et = 0;

    if((end.tv_nsec - init.tv_nsec) < 0){
            et = (end.tv_sec - init.tv_sec - 1) + (1000000000 + end.tv_nsec - init.tv_nsec);
    } else{
            et = (end.tv_sec - init.tv_sec) + (end.tv_nsec - init.tv_nsec);
    }

    return et;
}

int writePointCloud(const char* file, const pointCloud* pc){
    FILE* pcfile = fopen(file, "w");
    if(!pcfile){perror ("Error opening file!\n"); return -1;}
    
    for(int i = 0; i < pc->numPts; i++){
        fprintf(pcfile, "%f %f %f\n", pc->x[i], pc->y[i], pc->z[i]);
    }
    
    fclose(pcfile);
    return 0;
}

statMetrics* calculateStatMetrics(const pointCloud* pc){
    statMetrics *sm = (statMetrics*)malloc(sizeof(statMetrics));
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    for(int i = 0; i < pc->numPts; i++){
        sm->minX = pc->x[0]; sm->maxX = pc->x[0];
        sm->minY = pc->y[0]; sm->maxY = pc->y[0];
        sm->minZ = pc->z[0]; sm->maxZ = pc->z[0];

        if(pc->x[i] < sm->minX) sm->minX = pc->x[i];
        if(pc->x[i] > sm->maxX) sm->maxX = pc->x[i];
        sumX += pc->x[i];

        if(pc->y[i] < sm->minY) sm->minY = pc->y[i];
        if(pc->y[i] > sm->maxY) sm->maxY = pc->y[i];
        sumY += pc->y[i];

        if(pc->z[i] < sm->minZ) sm->minZ = pc->z[i];
        if(pc->z[i] > sm->maxZ) sm->maxZ = pc->z[i];
        sumZ += pc->z[i];
    }

    sm->avgX = sumX/pc->numPts;
    sm->avgY = sumY/pc->numPts;
    sm->avgZ = sumZ/pc->numPts;

    // variance (needed for standard deviation)
    float varX = 0;
    float varY = 0;
    float varZ = 0;

    for(int i = 0; i < pc->numPts; i++){
        varX += pow(pc->x[i] - sm->avgX, 2)/pc->numPts;
        varY += pow(pc->y[i] - sm->avgY, 2)/pc->numPts;
        varZ += pow(pc->z[i] - sm->avgZ, 2)/pc->numPts;
    }

    // standard deviation
    sm->stdX = sqrt(varX);
    sm->stdY = sqrt(varY);
    sm->stdZ = sqrt(varZ);

    return sm;
}


// ASSIGNMENT FUNCTIONS

pointCloud* readPointCloud(const char* file){
    FILE* pcfile = fopen(file, "r");
    if(!pcfile){perror ("Error opening file!\n"); return NULL;}

    int numPts = 0;
    char buf[BUF_SIZE];

    // calculate number of points (lines) first. this makes it
    // possible to allocate space for each dynamic array and
    // makes it easier to scan each line with fscanf
    while(fgets(buf, sizeof(buf), pcfile) != NULL){
        numPts++;
    }
    rewind(pcfile);

    // allocate space for the pointcloud and each dynamic array
    pointCloud* pc = (pointCloud*)malloc(sizeof(pointCloud));
    pc->x = (float*)malloc(sizeof(float)*numPts);
    pc->y = (float*)malloc(sizeof(float)*numPts);
    pc->z = (float*)malloc(sizeof(float)*numPts);
    pc->numPts = numPts;

    // process each line, storing xyz values to pc
    for(int i = 0; i < pc->numPts; i++){
        // check for errors and fscanf each line
        int retVal = 0;
        retVal = fscanf(pcfile, "%f %f %f", &pc->x[i], &pc->y[i], &pc->z[i]);
        if(retVal != 3){
            fprintf(stderr, "Error scanning line\n");
            free(pc->x);
            free(pc->y);
            free(pc->z);
            free(pc);
            fclose(pcfile);
            return NULL;
        }

    }

    return pc;
}


int preProcessPointCloud(pointCloud* pc){
    // (a)/(b)- remove all points with negative x values
    int aux_i = 0;

    for(int i = 0; i < pc->numPts; i++){
        if(pc->x[i] >= 0){
            pc->x[aux_i] = pc->x[i];
            pc->y[aux_i] = pc->y[i];
            pc->z[aux_i] = pc->z[i];
            aux_i++;
        }
    }

    pc->numPts = aux_i;

    // reallocate space to remove unutilized points
    pc->x = (float*)realloc(pc->x, aux_i*sizeof(float));
    pc->y = (float*)realloc(pc->y, aux_i*sizeof(float));
    pc->z = (float*)realloc(pc->z, aux_i*sizeof(float));


    // (c)- remove outliers that aren't ground/road
    // remove outliers based on height
    statMetrics* sm = calculateStatMetrics(pc);

    aux_i = 0;
    for(int i = 0; i < pc->numPts; i++){
        if(fabs(pc->z[i] - sm->avgZ) <= 1 * sm->stdZ){
            pc->x[aux_i] = pc->x[i];
            pc->y[aux_i] = pc->y[i];
            pc->z[aux_i] = pc->z[i];
            aux_i++;
        }
    }

    pc->numPts = aux_i;

    pc->x = (float*)realloc(pc->x, aux_i*sizeof(float));
    pc->y = (float*)realloc(pc->y, aux_i*sizeof(float));
    pc->z = (float*)realloc(pc->z, aux_i*sizeof(float));

    // remove outliers based on X/Y distance
    float maxX = 25.0;
    float maxY = 15.0;

    aux_i = 0;
    for(int i = 0; i < pc->numPts; i++){
        if(fabs(pc->x[i]) <= maxX && fabs(pc->y[i]) <= maxY){
            pc->x[aux_i] = pc->x[i];
            pc->y[aux_i] = pc->y[i];
            pc->z[aux_i] = pc->z[i];
            aux_i++;
        }
    }

    pc->numPts = aux_i;

    pc->x = (float*)realloc(pc->x, aux_i*sizeof(float));
    pc->y = (float*)realloc(pc->y, aux_i*sizeof(float));
    pc->z = (float*)realloc(pc->z, aux_i*sizeof(float));

    return 0;
}


int processDrivableAreaPointCloud(pointCloud* pc){
    float gridSize = 1.0;
    float minZThresh = -1.5;
    float maxZThresh = 1.5;
    float maxZDiff = 1.0;

    int gridXCount = (int)(50 / gridSize); // X grid cells
    int gridYCount = (int)(30 / gridSize); // Y grid cells

    // arrays to store min/max Z values for each grid cell
    float* minZGrid = (float*)malloc(gridXCount * gridYCount * sizeof(float));
    float* maxZGrid = (float*)malloc(gridXCount * gridYCount * sizeof(float));

    // initialize to +infinity and -infinity so the first data point is properly loaded
    for (int i = 0; i < gridXCount * gridYCount; i++) {
        minZGrid[i] = INFINITY;
        maxZGrid[i] = -INFINITY;
    }

    // calculate min and max Z per cell
    for (int i = 0; i < pc->numPts; i++) {
        int gridX = (int)((pc->x[i] + 25) / gridSize);
        int gridY = (int)((pc->y[i] + 15) / gridSize);

        if (gridX >= 0 && gridX < gridXCount && gridY >= 0 && gridY < gridYCount) {
            int gridIndex = gridY * gridXCount + gridX;
            if (pc->z[i] < minZGrid[gridIndex]) minZGrid[gridIndex] = pc->z[i];
            if (pc->z[i] > maxZGrid[gridIndex]) maxZGrid[gridIndex] = pc->z[i];
        }
    }

    // identify cells considered driveable according to Z values
    int aux_i = 0;
    for (int i = 0; i < pc->numPts; i++) {
        int gridX = (int)((pc->x[i] + 25) / gridSize);
        int gridY = (int)((pc->y[i] + 15) / gridSize);

        if (gridX >= 0 && gridX < gridXCount && gridY >= 0 && gridY < gridYCount) {
            int gridIndex = gridY * gridXCount + gridX;
            float zMin = minZGrid[gridIndex];
            float zMax = maxZGrid[gridIndex];

            // check if point is driveable
            if (zMin >= minZThresh && zMax <= maxZThresh && (zMax - zMin) <= maxZDiff) {
                pc->x[aux_i] = pc->x[i];
                pc->y[aux_i] = pc->y[i];
                pc->z[aux_i] = pc->z[i];
                aux_i++;
            }
        }
    }

    pc->numPts = aux_i;
    pc->x = (float*)realloc(pc->x, aux_i * sizeof(float));
    pc->y = (float*)realloc(pc->y, aux_i * sizeof(float));
    pc->z = (float*)realloc(pc->z, aux_i * sizeof(float));

    free(minZGrid);
    free(maxZGrid);

    return 0;
}


int main(int argc, char* argv[]){
    // lock paging & set highet priority (RTS)
    mlockall(MCL_CURRENT | MCL_FUTURE);
    int pid = getpid();
    setpriority(PRIO_PROCESS, pid, -20);

    pointCloud* pc1 = readPointCloud("support_material/point_cloud1.txt");
    pointCloud* pc2 = readPointCloud("support_material/point_cloud2.txt");
    pointCloud* pc3 = readPointCloud("support_material/point_cloud3.txt");

    statMetrics* sm1 = calculateStatMetrics(pc1);
    statMetrics* sm2 = calculateStatMetrics(pc2);
    statMetrics* sm3 = calculateStatMetrics(pc3);

    printf("POINT CLOUD 1:\n");
    printf("X: min = %f, max = %f, average = %f, standard deviation = %f\n", sm1->minX, sm1->maxX, sm1->avgX, sm1->stdX);
    printf("Y: min = %f, max = %f, average = %f, standard deviation = %f\n", sm1->minY, sm1->maxY, sm1->avgY, sm1->stdY);
    printf("Z: min = %f, max = %f, average = %f, standard deviation = %f\n", sm1->minZ, sm1->maxZ, sm1->avgZ, sm1->stdZ);

    printf("POINT CLOUD 2:\n");
    printf("X: min = %f, max = %f, average = %f, standard deviation = %f\n", sm2->minX, sm2->maxX, sm2->avgX, sm2->stdX);
    printf("Y: min = %f, max = %f, average = %f, standard deviation = %f\n", sm2->minY, sm2->maxY, sm2->avgY, sm2->stdY);
    printf("Z: min = %f, max = %f, average = %f, standard deviation = %f\n", sm2->minZ, sm2->maxZ, sm2->avgZ, sm2->stdZ);

    printf("POINT CLOUD 3:\n");
    printf("X: min = %f, max = %f, average = %f, standard deviation = %f\n", sm3->minX, sm3->maxX, sm3->avgX, sm3->stdX);
    printf("Y: min = %f, max = %f, average = %f, standard deviation = %f\n", sm3->minY, sm3->maxY, sm3->avgY, sm3->stdY);
    printf("Z: min = %f, max = %f, average = %f, standard deviation = %f\n", sm3->minZ, sm3->maxZ, sm3->avgZ, sm3->stdZ);

    printf("\n");

    // calculate execution time of all 3 functions for all 3 point clouds at once
    struct timespec init, end;

    clock_gettime(CLOCK_MONOTONIC, &init);
    pc1 = readPointCloud("support_material/point_cloud1.txt");
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (readPointCloud(pc1)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    clock_gettime(CLOCK_MONOTONIC, &init);
    pc2 = readPointCloud("support_material/point_cloud2.txt");
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (readPointCloud(pc2)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    clock_gettime(CLOCK_MONOTONIC, &init);
    pc3 = readPointCloud("support_material/point_cloud3.txt");
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (readPointCloud(pc3)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));
    
    printf("\n");

    clock_gettime(CLOCK_MONOTONIC, &init);
    preProcessPointCloud(pc1);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (preProcessPointCloud(pc1)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    clock_gettime(CLOCK_MONOTONIC, &init);
    preProcessPointCloud(pc2);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (preProcessPointCloud(pc2)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    clock_gettime(CLOCK_MONOTONIC, &init);
    preProcessPointCloud(pc3);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (preProcessPointCloud(pc3)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    printf("\n");
    
    clock_gettime(CLOCK_MONOTONIC, &init);
    processDrivableAreaPointCloud(pc1);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (processDrivableAreaPointCloud(pc1)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    clock_gettime(CLOCK_MONOTONIC, &init);
    processDrivableAreaPointCloud(pc2);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (processDrivableAreaPointCloud(pc2)): %d (us)", (int)(calculateElapsedTime(init, end)/1e3));

    clock_gettime(CLOCK_MONOTONIC, &init);
    processDrivableAreaPointCloud(pc3);
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("\nExecution time (processDrivableAreaPointCloud(pc3)): %d (us)\n", (int)(calculateElapsedTime(init, end)/1e3));


    writePointCloud("support_material/point_cloud1_preprocessed.txt", pc1);
    writePointCloud("support_material/point_cloud2_preprocessed.txt", pc2);
    writePointCloud("support_material/point_cloud3_preprocessed.txt", pc3);

    munlockall();
    return 0;
}
