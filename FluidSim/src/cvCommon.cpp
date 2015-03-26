#include <tuple>
#include <vector>
#include "stdint.h"

using namespace std;

class CommonFunctionLibrary
{

	//first is the depth value, second is the index (only set if depth value is valid)
	template <typename T>
	std::vector<pair<bool, uint16_t>> getSquareFromTopLeft(std::vector<T> depthData, int x, int y, int w, int h, float farClipZ)
	{
		std::vector<pair<bool, uint16_t>> window = { { false, 0 }, { false, 0 }, { false, 0 }, { false, 0 } };

		window[0].first = depthData[x + (y * w)] != 0 && depthData[x + (y * w)] < farClipZ; //set to the top left pixel
		window[0].second = x + (y * w);

		if (x == w - 1 && y == h - 1) //bottom right
		{
			window[1].first = false;
			window[2].first = false;
			window[3].first = false;
		}
		else if (x == w - 1 || y == h - 1) //bottom right)
		{
			if (x == w - 1) //right
			{
				window[1].first = false;
				window[2].first = depthData[x + ((y + 1) * w)] != 0 && depthData[x + ((y + 1) * w)] < farClipZ;
				window[2].second = x + ((y + 1) * w);
				window[3].first = false;
			}

			if (y == h - 1) //bottom
			{
				window[1].first = depthData[(x + 1) + (y * w)] != 0 && depthData[(x + 1) + (y * w)] < farClipZ;
				window[1].second = (x + 1) + (y * w);
				window[2].first = false;
				window[3].first = false;
			}
		}
		else
		{
			window[1].first = depthData[(x + 1) + (y * w)] != 0 && depthData[(x + 1) + (y * w)] < farClipZ;
			window[1].second = (x + 1) + (y * w);
			window[2].first = depthData[x + ((y + 1) * w)] != 0 && depthData[x + ((y + 1) * w)] < farClipZ;
			window[2].second = x + ((y + 1) * w);
			window[3].first = depthData[(x + 1) + ((y + 1) * w)] != 0 && depthData[(x + 1) + ((y + 1) * w)] < farClipZ;
			window[3].second = (x + 1) + ((y + 1) * w);
		}


		return window;
	}

	///// <summary>
	///// Pulling a window of pixels out of an image. All arrays involved are 1-dimensional.
	///// </summary>
	///// <param name="pixels">pixel array, each byte should have a grayscale pixel color, so the size of the array should be (width * height)</param>
	///// <param name="x">middle x position of window</param>
	///// <param name="y">middle y position of window</param>
	///// <param name="width">width of the image</param>
	///// <param name="height">height of the image</param>
	///// <param name="window_size">dimensions of the window</param>
	///// <returns></returns>
	//template <typename T>
	//std::vector<T> getWindowFromMiddle(std::vector<T> &PreviouslyComputedWindow, std::vector<T> depthData, int x, int y, int w, int h, int window_size = 3)
	//{
	//	//window size is only 3x3 for now, this is a 9-byte array of depthData
	//	//0, 1, 2
	//	//3, 4, 5
	//	//6, 7, 8
	//	std::vector<T> window(window_size * window_size);
	//
	//	//Corner Cases
	//	if (x == 0 && y == 0) //top left is middle
	//	{
	//		window[0] = depthData[x + 1 + ((y + 1) * w)]; //mirror top left with bottom right
	//		window[1] = depthData[x + ((y + 1) * w)]; //mirror top middle with bottom middle
	//		window[2] = depthData[x + 1 + ((y + 1) * w)]; //mirror top right with bottom right
	//
	//		window[3] = depthData[x + 1 + (y * w)]; //mirror middle-left with middle-right
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x + 1 + (y * w)];
	//
	//		window[6] = depthData[x + 1 + ((y + 1) * w)]; //mirror bottom-left with bottom-right
	//		window[7] = depthData[x + ((y + 1) * w)];
	//		window[8] = depthData[x + 1 + ((y + 1) * w)];
	//	}
	//	else if (x == w - 1 && y == 0) //top right is middle
	//	{
	//		window[0] = depthData[x - 1 + ((y + 1) * w)]; //0 <- 6
	//		window[1] = depthData[x + ((y + 1) * w)]; //1 <- 7
	//		window[2] = depthData[x - 1 + ((y + 1) * w)]; //2 <- 6
	//
	//		window[3] = depthData[x - 1 + (y * w)];
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x - 1 + (y * w)]; //5 <- 3
	//
	//		window[6] = depthData[x - 1 + ((y + 1) * w)];
	//		window[7] = depthData[x + ((y + 1) * w)];
	//		window[8] = depthData[x - 1 + ((y + 1) * w)]; //8 <- 6
	//	}
	//	else if (x == 0 && y == h - 1) //bottom left is middle
	//	{
	//		window[0] = depthData[x + 1 + ((y - 1) * w)]; //0 <- 2
	//		window[1] = depthData[x + ((y - 1) * w)];
	//		window[2] = depthData[x + 1 + ((y - 1) * w)];
	//
	//		window[3] = depthData[x + 1 + (y * w)]; //3 <- 5
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x + 1 + (y * w)];
	//
	//		window[6] = depthData[x + 1 + ((y - 1) * w)]; //6 <- 2
	//		window[7] = depthData[x + ((y - 1) * w)]; //7 <- 1
	//		window[8] = depthData[x + 1 + ((y - 1) * w)]; //8 <- 2
	//	}
	//	else if (x == w - 1 && y == h - 1) //bottom right is middle
	//	{
	//		window[0] = depthData[x - 1 + ((y - 1) * w)];
	//		window[1] = depthData[x + ((y - 1) * w)];
	//		window[2] = depthData[x - 1 + ((y - 1) * w)]; //2 <- 0
	//
	//		window[3] = depthData[x - 1 + (y * w)];
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x - 1 + (y * w)]; //5 <- 3
	//
	//		window[6] = depthData[x - 1 + ((y - 1) * w)]; //6 <- 0
	//		window[7] = depthData[x + ((y - 1) * w)]; //7 <- 1
	//		window[8] = depthData[x - 1 + ((y - 1) * w)]; //8 <- 0
	//	}
	//
	//	//0, 1, 2
	//	//3, 4, 5
	//	//6, 7, 8
	//
	//	//Side Cases
	//	else if (y == 0) // middle is in top row
	//	{
	//		window[0] = depthData[x - 1 + ((y + 1) * w)]; //0 <- 6
	//		window[1] = depthData[x + ((y + 1) * w)]; //1 <- 7
	//		window[2] = depthData[x + 1 + ((y + 1) * w)]; //2 <- 8
	//
	//		window[3] = depthData[x - 1 + (y * w)];
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x + 1 + (y * w)];
	//
	//		window[6] = depthData[x - 1 + ((y + 1) * w)];
	//		window[7] = depthData[x + ((y + 1) * w)];
	//		window[8] = depthData[x + 1 + ((y + 1) * w)];
	//	}
	//	else if (x == 0) // middle is in left-most column
	//	{
	//		window[0] = depthData[x + 1 + ((y - 1) * w)]; //0 <- 2
	//		window[1] = depthData[x + ((y - 1) * w)];
	//		window[2] = depthData[x + 1 + ((y - 1) * w)];
	//
	//		window[3] = depthData[x + 1 + (y * w)]; //3 <- 5
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x + 1 + (y * w)];
	//
	//		window[6] = depthData[x + 1 + ((y + 1) * w)]; //6 <- 8
	//		window[7] = depthData[x + ((y + 1) * w)];
	//		window[8] = depthData[x + 1 + ((y + 1) * w)];
	//	}
	//	else if (y == h - 1) //middle is in bottom row
	//	{
	//		window[0] = depthData[x - 1 + ((y - 1) * w)];
	//		window[1] = depthData[x + ((y - 1) * w)];
	//		window[2] = depthData[x + 1 + ((y - 1) * w)];
	//
	//		window[3] = depthData[x - 1 + (y * w)];
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x + 1 + (y * w)];
	//
	//		window[6] = depthData[x - 1 + ((y - 1) * w)]; //6 <- 0
	//		window[7] = depthData[x + ((y - 1) * w)]; //7 <- 1
	//		window[8] = depthData[x + 1 + ((y - 1) * w)]; //8 <- 2
	//	}
	//	else if (x == w - 1) //middle is in right-most column
	//	{
	//		window[0] = depthData[x - 1 + ((y - 1) * w)];
	//		window[1] = depthData[x + ((y - 1) * w)];
	//		window[2] = depthData[x - 1 + ((y - 1) * w)]; //2 <- 0
	//
	//		window[3] = depthData[x - 1 + (y * w)];
	//		window[4] = depthData[x + (y * w)];
	//		window[5] = depthData[x - 1 + (y * w)]; //5 <- 3
	//
	//		window[6] = depthData[x - 1 + ((y + 1) * w)];
	//		window[7] = depthData[x + ((y + 1) * w)];
	//		window[8] = depthData[x - 1 + ((y + 1) * w)]; //8 <- 6
	//	}
	//
	//	//Middle Not a Border
	//	else
	//	{    //old
	//		//X, 1, 2 -- move 1, 2 -> 0, 1
	//		//X, 4, 5 -- move 4, 5 -> 3, 4
	//		//X, 7, 8 -- move 7, 8 -> 6, 7
	//		if (PreviouslyComputedWindow.size() == (w * h) && x > 1 && y > 1 && x < w - 2 && y < h - 2)
	//		{ //x just incremented by 1, and we're not next to a border, so we should be able to use the previously computed window to avoid re-computation
	//
	//			//All this saves is a little bit of time calculating indeces as shown in the else statement here... it doesn't save on the computations for calculating pixel values
	//			window[0] = PreviouslyComputedWindow[1];
	//			window[1] = PreviouslyComputedWindow[2];
	//			window[2] = depthData[x + 1 + ((y - 1) * w)];
	//
	//			window[3] = PreviouslyComputedWindow[4];
	//			window[4] = PreviouslyComputedWindow[5];
	//			window[5] = depthData[x + 1 + (y * w)];
	//
	//			window[6] = PreviouslyComputedWindow[7];
	//			window[7] = PreviouslyComputedWindow[8];
	//			window[8] = depthData[x + 1 + ((y + 1) * w)];
	//		}
	//		else
	//		{ //compute the window from scratch
	//			window[0] = depthData[x - 1 + ((y - 1) * w)];
	//			window[1] = depthData[x + ((y - 1) * w)];
	//			window[2] = depthData[x + 1 + ((y - 1) * w)];
	//
	//			window[3] = depthData[x - 1 + (y * w)];
	//			window[4] = depthData[x + (y * w)];
	//			window[5] = depthData[x + 1 + (y * w)];
	//
	//			window[6] = depthData[x - 1 + ((y + 1) * w)];
	//			window[7] = depthData[x + ((y + 1) * w)];
	//			window[8] = depthData[x + 1 + ((y + 1) * w)];
	//		}
	//	}
	//
	//	PreviouslyComputedWindow = window;
	//
	//	window = TransposeWindow(window);
	//
	//	return window;
	//}


	////imgDepths is the subsampled back Z's and scales
	//void HP_WaitingRTApp::SetFlowWithBlurKernel(int w, int h)
	//{
	//	std::vector<float> blurredImage;
	//	std::vector<float> PreviouslyComputedWindow; //used to speed up window computation
	//
	//
	//	//average out the array data first.
	//	for (int y = 0; y < h; y++)
	//	{
	//		for (int x = 0; x < w; x++)
	//		{
	//
	//			float averageZ = 0;
	//			if (imgDepths[x + (y * w)] != 0)
	//			{
	//				std::vector<float> currentWindow = getWindowFromMiddle<float>(PreviouslyComputedWindow, imgDepths, x, y, w, h, 3); //3x3 window size
	//				int numUsedInWindow = 0;
	//				for (int i = 0; i < currentWindow.size(); i++)
	//				{
	//					if (currentWindow[i] != 0)
	//					{
	//						averageZ += currentWindow[i];
	//						numUsedInWindow++;
	//					}
	//				}
	//
	//				averageZ /= (float)numUsedInWindow;
	//			}
	//			else
	//			{
	//				PreviouslyComputedWindow.clear();
	//			}
	//
	//			blurredImage.push_back(averageZ);
	//		}
	//	}
	//
	//	return blurredImage;
	//}

};