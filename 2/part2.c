#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>
#include <pthread.h>

#define BUF_SIZE 64
#define NUM_THREADS 3

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

pthread_mutex_t mutex;
pointCloud sharedPointCloud;
int processingDone[NUM_THREADS] = {0};
struct timespec sleep1ms = {0, 1000000};

void* threadFunc1(void* arg) {
    struct timespec init, end;
    clock_gettime(CLOCK_MONOTONIC, &init);
    
    const char* file = (const char*) arg;

    pthread_mutex_lock(&mutex);
    printf("Thread 1: reading point cloud\n");
    pointCloud* pc = readPointCloud(file);
    if (!pc) {
        fprintf(stderr, "Error reading point cloud file\n");
        pthread_mutex_unlock(&mutex);
        pthread_exit(NULL);
    }
    
    sharedPointCloud = *pc;
    pthread_mutex_unlock(&mutex);

    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Thread 1 time: %d (us)\n", (int)(calculateElapsedTime(init, end)/1e3));

    processingDone[0] = 1;
    pthread_exit(NULL);
}

void* threadFunc2(void* arg) {
    while (!processingDone[0]) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep1ms, NULL); // wait for thread 1
    }

    struct timespec init, end;
    clock_gettime(CLOCK_MONOTONIC, &init);

    pthread_mutex_lock(&mutex);
    printf("Thread 2: preprocessing point cloud\n");
    
    preProcessPointCloud(&sharedPointCloud);
    pthread_mutex_unlock(&mutex);

    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Thread 2 time: %d (us)\n", (int)(calculateElapsedTime(init, end)/1e3));

    processingDone[1] = 1;
    pthread_exit(NULL);
}

void* threadFunc3(void* arg) {
    while (!processingDone[1]) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep1ms, NULL); // wait for thread 2
    }

    struct timespec init, end;
    clock_gettime(CLOCK_MONOTONIC, &init);

    pthread_mutex_lock(&mutex);
    printf("Thread 3: Identifying ground points\n");
    
    processDrivableAreaPointCloud(&sharedPointCloud);
    pthread_mutex_unlock(&mutex);

    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Thread 3 Time: %d (us)\n", (int)(calculateElapsedTime(init, end)/1e3));

    processingDone[2] = 1;
    pthread_exit(NULL);
}

int main(int argc, char* argv[]){
    // lock paging & set highet priority (RTS)
    mlockall(MCL_CURRENT | MCL_FUTURE);
    int pid = getpid();
    setpriority(PRIO_PROCESS, pid, -20);

    pthread_t threads[NUM_THREADS];
    pthread_attr_t attr[NUM_THREADS];
    
    pthread_attr_init(&attr[0]);
    pthread_attr_init(&attr[1]);
    pthread_attr_init(&attr[2]);
    
    pthread_attr_setschedpolicy(&attr[0], SCHED_FIFO);
    pthread_attr_setschedpolicy(&attr[1], SCHED_FIFO);
    pthread_attr_setschedpolicy(&attr[2], SCHED_FIFO);
    
    pthread_mutex_init(&mutex, NULL);

    pthread_create(&threads[0], &attr[0], threadFunc1, "support_material/point_cloud1.txt");
    pthread_create(&threads[1], &attr[1], threadFunc2, NULL);
    pthread_create(&threads[2], &attr[2], threadFunc3, NULL);

    for (int i = 0; i < NUM_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    pthread_mutex_destroy(&mutex);
    printf("All threads executed successfully\n");

    munlockall();
    return 0;
}
