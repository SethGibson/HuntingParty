#ifndef DEPTH_FILTER_INTERFACE_H
#define DEPTH_FILTER_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	//MEANSHIFT and BILATERAL are OpenCL Kernels and may crash with incorrect OpenCL runtime requirements.

typedef enum {
	DEPTH_FILTER_BILATERAL = 0,		//supports DEPTH_FILTER_PROP_SIGMA
	DEPTH_FILTER_GUIDED,			//supports DEPTH_FILTER_PROP_RADIUS | DEPTH_FILTER_PROP_SIGMA | DEPTH_FILTER_PROP_REQ_DENSITY
	DEPTH_FILTER_MEANSHIFT,			//supports DEPTH_FILTER_PROP_RADIUS | DEPTH_FILTER_PROP_SIGMA | DEPTH_FILTER_PROP_SEG_SIZE
	DEPTH_FILTER_KNN,				//supports DEPTH_FILTER_PROP_SIGMA | DEPTH_FILTER_PROP_REQ_DENSITY | DEPTH_FILTER_PROP_DIST_THRESH | DEPTH_FILTER_PROP_SCALES

	DEPTH_FILTER_NOCOLOR = 16		//for the future: filters that do not use color
} FilterType ;

typedef enum {
	DEPTH_FILTER_PROP_RADIUS = 0,	//search radius for local filters
	DEPTH_FILTER_PROP_SIGMA,		//weight error in guided space
	DEPTH_FILTER_PROP_REQ_DENSITY,	//how dense should a patch be to be filled in
	DEPTH_FILTER_PROP_SCALES,		//number of scales (for multi-scale approaches)
	DEPTH_FILTER_PROP_DIST_THRESH,	//in normalized space, how far in N-D space is too far?
	DEPTH_FILTER_PROP_SEG_SIZE,		//smallest segment size

	DEPTH_FILTER_PROP_MAX			//final propety, useful for iterating
} FilterProperty;

typedef struct depthf_Handle_ * depthf_Handle; /* depthf_Handle is now an opaque handle type */

depthf_Handle	depthf_Create					(FilterType type, int width, int height); /* Function to construct */
int				depthf_Filter					(depthf_Handle object, const uint16_t * input, const uint8_t * guide, uint16_t * output);
void			depthf_Delete					(depthf_Handle object);

//properties!
int				depthf_SetProperty				(depthf_Handle object, FilterProperty key, float  value);
int				depthf_GetProperty				(depthf_Handle object, FilterProperty key, float *value);

//	 "int" return codes are
//		 0 = success

#ifdef __cplusplus
}
#endif

#endif
