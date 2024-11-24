#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define BUF_SIZE 64

typedef struct{
    float* x;
    float* y;
    float* z;

    int numPts;
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
} pointCloud;

// AUXILIARY FUNCTIONS
int calculateMetricsPointCloud(pointCloud* pc){
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    for(int i = 0; i < pc->numPts; i++){
        pc->minX = pc->x[0]; pc->maxX = pc->x[0];
        pc->minY = pc->y[0]; pc->maxY = pc->y[0];
        pc->minZ = pc->z[0]; pc->maxZ = pc->z[0];

        if(pc->x[i] < pc->minX) pc->minX = pc->x[i];
        if(pc->x[i] > pc->maxX) pc->maxX = pc->x[i];
        sumX += pc->x[i];

        if(pc->y[i] < pc->minY) pc->minY = pc->y[i];
        if(pc->y[i] > pc->maxY) pc->maxY = pc->y[i];
        sumY += pc->y[i];

        if(pc->z[i] < pc->minZ) pc->minZ = pc->z[i];
        if(pc->z[i] > pc->maxZ) pc->maxZ = pc->z[i];
        sumZ += pc->z[i];
    }

    // calculate statistical metrics

    // mean (average)
    pc->avgX = sumX/pc->numPts;
    pc->avgY = sumY/pc->numPts;
    pc->avgZ = sumZ/pc->numPts;

    // variance (needed for standard deviation)
    float varX, varY, varZ;
    for(int i = 0; i < pc->numPts; i++){
        varX += pow(pc->x[i] - pc->avgX, 2)/pc->numPts;
        varY += pow(pc->y[i] - pc->avgY, 2)/pc->numPts;
        varZ += pow(pc->z[i] - pc->avgZ, 2)/pc->numPts;
    }

    // standard deviation
    pc->stdX = sqrt(varX);
    pc->stdY = sqrt(varY);
    pc->stdZ = sqrt(varZ);

    return 0;
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
    for(int i = 0; i < numPts; i++){
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

    calculateMetricsPointCloud(pc);
    return pc;
}

int preProcessPointCloud(pointCloud* pc){
    // (a)- remove all points with negative x values
    for(int i = 0; i < pc->numPts; i++){
        if(pc->x[i] < 0){
            float* tmpX = (float*)malloc(sizeof(float)*(pc->numPts-1));
            float* tmpY = (float*)malloc(sizeof(float)*(pc->numPts-1));
            float* tmpZ = (float*)malloc(sizeof(float)*(pc->numPts-1));

            //memmove(tmpX, pc, (i+1)*sizeof(float));
            //memmove(tmpX+i, (pc)+(i+1), (pc->numPts-i)*sizeof(float));

            //memmove(tmpY, pc, (i+1)*sizeof(float));
            //memmove(tmpY+i, (pc)+(i+1), (pc->numPts-i)*sizeof(float));

            //memmove(tmpZ, pc, (i+1)*sizeof(float));
            //memmove(tmpZ+i, (pc)+(i+1), (pc->numPts-i)*sizeof(float));

            pc->x = tmpX;
            pc->y = tmpY;
            pc->z = tmpZ;

            pc->numPts--;

            free(tmpX);
            free(tmpY);
            free(tmpZ);
        }
    }

    return 0;
}

int main(int argc, char* argv[]){
    pointCloud* pc1 = readPointCloud("support_material/point_cloud1.txt");
    pointCloud* pc2 = readPointCloud("support_material/point_cloud2.txt");
    pointCloud* pc3 = readPointCloud("support_material/point_cloud3.txt");

    //preProcessPointCloud(pc1);
    //preProcessPointCloud(pc2);
    //preProcessPointCloud(pc3);

    printf("POINT CLOUD 1:\n");
    printf("X: min = %f, max = %f, average = %f, standard deviation = %f\n", pc1->minX, pc1->maxX, pc1->avgX, pc1->stdX);
    printf("Y: min = %f, max = %f, average = %f, standard deviation = %f\n", pc1->minY, pc1->maxY, pc1->avgY, pc1->stdY);
    printf("Z: min = %f, max = %f, average = %f, standard deviation = %f\n", pc1->minZ, pc1->maxZ, pc1->avgZ, pc1->stdZ);

    printf("POINT CLOUD 2:\n");
    printf("X: min = %f, max = %f, average = %f, standard deviation = %f\n", pc2->minX, pc2->maxX, pc2->avgX, pc2->stdX);
    printf("Y: min = %f, max = %f, average = %f, standard deviation = %f\n", pc2->minY, pc2->maxY, pc2->avgY, pc2->stdY);
    printf("Z: min = %f, max = %f, average = %f, standard deviation = %f\n", pc2->minZ, pc2->maxZ, pc2->avgZ, pc2->stdZ);

    printf("POINT CLOUD 3:\n");
    printf("X: min = %f, max = %f, average = %f, standard deviation = %f\n", pc3->minX, pc3->maxX, pc3->avgX, pc3->stdX);
    printf("Y: min = %f, max = %f, average = %f, standard deviation = %f\n", pc3->minY, pc3->maxY, pc3->avgY, pc3->stdY);
    printf("Z: min = %f, max = %f, average = %f, standard deviation = %f\n", pc3->minZ, pc3->maxZ, pc3->avgZ, pc3->stdZ);

    return 0;
}

