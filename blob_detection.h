#ifndef __BLOB_DETECTION__H__
#define __BLOB_DETECTION__H__
#include "disjoint.h"
#include "eecs467_util.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define COLOR_THRESHOLD 100.0
#define TEMPLATE_PX 0xff32ffff
#define SHOW_PX 0xffe127ff
#define MIN_PXS 125
#define MAX_PXS 400
#define MAX_NUM_NEIGHBORS 8
#define MAX_NUM_BALLS 1500

typedef struct ball ball_t;
struct ball {
    double x, y;
    int num_px;
};

unsigned int color_dist(uint32_t p1, uint32_t p2);

unsigned int is_ball(uint32_t px);


//Returns number of neighbors, modifies neighbors array to
//contain their position in the buf
unsigned int getNeighbors(image_u32_t *im, int x, int y, 
	int neighbors[MAX_NUM_NEIGHBORS]);

void getNLabels(int n_labels[], int labels[], int neighbors[], int
	len_neighbors);

//Requires len_labels >= 1
int minLabel(int n_labels[], int len_labels);


void unionLabels(Set *links[MAX_NUM_BALLS], int n_labels[MAX_NUM_NEIGHBORS],
	int len_neighbors); 

int blob_detection(image_u32_t *im, ball_t *final_balls);


#endif
