/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>


OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color);
uint8 otsuThreshold(void);
void drawHist(uint32* p_hist);

void ProcessFrame(uint8 *pInputImg)
{
	int c, r;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);


	struct OSC_PICTURE Pic1, Pic2;
	struct OSC_VIS_REGIONS ImgRegions;
	uint8 threshold = data.ipc.state.nThreshold;

	if(threshold == 0)
	{
		threshold = otsuThreshold();
	}
	else
	{
		//correction factor
		threshold = (threshold*255)/100;
	}


	for(r = 0; r < siz; r += nc)
	{
		for(c = 0; c < nc; c++)
		{
			data.u8TempImage[THRESHOLD][r+c] = (data.u8TempImage[GRAYSCALE][r+c] < threshold ? 255 : 0);
		}
	}

	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)/* we skip the first and last column */
		{
			unsigned char* p = &data.u8TempImage[THRESHOLD][r+c];
			data.u8TempImage[EROSION][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
											 *(p-1)    & *p      & *(p+1)    &
											 *(p+nc-1) & *(p+nc) & *(p+nc+1);
		}
	}

	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)/* we skip the first and last column */
		{
			unsigned char* p = &data.u8TempImage[EROSION][r+c];
			data.u8TempImage[DILATION][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
											  *(p-1)    | *p      | *(p+1)    |
											  *(p+nc-1) | *(p+nc) | *(p+nc+1);
		}
	}

	//wrap image DILATION in picture struct
	Pic1.data = data.u8TempImage[DILATION];
	Pic1.width = nc;
	Pic1.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
	Pic1.type = OSC_PICTURE_GREYSCALE;
	//as well as EROSION (will be used as output)
	Pic2.data = data.u8TempImage[EROSION];
	Pic2.width = nc;
	Pic2.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
	Pic2.type = OSC_PICTURE_BINARY;//probably has no consequences
	//have to convert to OSC_PICTURE_BINARY which has values 0x01 (and not 0xff)
	OscVisGrey2BW(&Pic1, &Pic2, 0x80, false);

	//now do region labeling and feature extraction
	OscVisLabelBinary( &Pic2, &ImgRegions);
	OscVisGetRegionProperties( &ImgRegions);

	Pic2.data = data.u8TempImage[GRAYSCALE];
	OscVisDrawBoundingBoxBW(&Pic2, &ImgRegions, 255);
	OscVisDrawBoundingBoxBW( &Pic1, &ImgRegions, 128);

	// Error Logging with OscLog
	//OscLog(level, "error, error, ...");

}

/**
 * \brief Method otsu.
 * \return Computed threshold.
 */
uint8 otsuThreshold(void)
{
	int c, r;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);

	uint32 Hist[256];		// 4 Bytes *256 = 1kB, absolute Histogram
	uint32 W0, W1, M0, M1;	// written in capitals, because these are absolute
							// values (relative would be: w0, w1, 'micro0', 'micro1')
	float temp;
	float sigma_B, sigma_B_max = 0;
	uint16 g, k, KBest = 0;	// not uint8, because g and can overflow in for loops

	memset(Hist, 0, sizeof(Hist));


	// Compute the (absolute) histogram
	for(r = 0; r < siz; r += nc)
	{
		for(c = 0; c < nc; c++)
		{
			Hist[data.u8TempImage[GRAYSCALE][r+c]] ++;		// increment Hist according to given gray-value
		}
	}

	// Compute (absolute) omega's and micro's for all values of k.
	for(k = 0; k < 256; k++)
	{
		W0 = 0;
		M0 = 0;
		W1 = 0;
		M1 = 0;
		for(g = 0; g <= k; g++)
		{
			W0 += Hist[g];
			M0 += Hist[g]*g;
		}
		for(g = k+1; g <= 255; g++)
		{
			W1 += Hist[g];
			M1 += Hist[g]*g;
		}
		temp = (((float)M0)/W0) - (((float)M1)/W1);
		sigma_B = W0 * W1 * temp*temp;	// without 1/N^2, because it is just a constant factor.

		if(sigma_B > sigma_B_max)
		{
			sigma_B_max = sigma_B;
			KBest = k;
		}
	}
	//printf("\nMethod of Otsu: Best k = %i", KBest);
	return KBest;
}

#if 0
/**
 * \brief draws a histogram into the image BACKGROUND.
 */
void drawHist(uint32* p_hist)
{
	int c, r;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[BACKGOUND]);

	for(r = 0; r < siz; r += nc)
	{
		for(c = 0; c < nc; c++)
		{
			//Hist[data.u8TempImage[GRAYSCALE][r+c]];		// increment Hist according to given gray-value
		}
	}
	return 40;
}
#endif

/* Drawing Function for Bounding Boxes; own implementation because Oscar only allows colored boxes; here in Gray value "Color"  */
/* should only be used for debugging purposes because we should not drawn into a gray scale image */
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color)
{
	 uint16 i, o;
	 uint8 *pImg = (uint8*)picIn->data;
	 const uint16 width = picIn->width;
	 for(o = 0; o < regions->noOfObjects; o++)//loop over regions
	 {
		 /* Draw the horizontal lines. */
		 for (i = regions->objects[o].bboxLeft; i < regions->objects[o].bboxRight; i += 1)
		 {
				 pImg[width * regions->objects[o].bboxTop + i] = Color;
				 pImg[width * (regions->objects[o].bboxBottom - 1) + i] = Color;
		 }

		 /* Draw the vertical lines. */
		 for (i = regions->objects[o].bboxTop; i < regions->objects[o].bboxBottom-1; i += 1)
		 {
				 pImg[width * i + regions->objects[o].bboxLeft] = Color;
				 pImg[width * i + regions->objects[o].bboxRight] = Color;
		 }
	 }
	 return SUCCESS;
}



