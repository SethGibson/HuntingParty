#include <DSAPI.h>
#include <DSAPI\DSCalibRectParametersUtil.h>
#include <memory>
#include <algorithm>
class RGBD
{
public:
	RGBD(DSAPI * ds, size_t hole_num = 4)
		:ds(ds), zw_(ds->zWidth()), zh_(ds->zHeight()), hn_(hole_num), coord(new uint16_t[2 * ds->zHeight()*ds->zWidth()])
	{}
	void getRGBAlignedZ(const uint16_t* zIn, uint16_t* zOut)
	{
		memset(coord.get(), 0, sizeof(uint16_t) * 2 * zw_*zh_);
		const auto rgb_width = ds->accessThird()->thirdWidth();
		const auto rgb_height = ds->accessThird()->thirdHeight();
		const uint16_t * zD = zIn;
		DSCalibIntrinsicsRectified zIntrin, thirdIntrin;
		double zToThirdTrans[3];
		ds->getCalibIntrinsicsZ(zIntrin);
		ds->accessThird()->getCalibIntrinsicsRectThird(thirdIntrin);
		ds->accessThird()->getCalibExtrinsicsZToRectThird(zToThirdTrans);


		for (int i = 0; i < zh_; i++) {
			for (int j = 0; j < zw_; j++) {
				if (zD[i*zw_ + j] != 0) {
					float zImage[] = { static_cast<float>(j), static_cast<float>(i), (float)zD[i*zw_ + j] }, zCamera[3], thirdCamera[3], thirdImage[2];
					DSTransformFromZImageToZCamera(zIntrin, zImage, zCamera);
					DSTransformFromZCameraToRectThirdCamera(zToThirdTrans, zCamera, thirdCamera);
					DSTransformFromThirdCameraToRectThirdImage(thirdIntrin, thirdCamera, thirdImage);
					int yCoord = static_cast<int>(thirdImage[1] + 0.5);
					int xCoord = static_cast<int>(thirdImage[0] + 0.5);
					if (xCoord >= 0 && xCoord < (int)thirdIntrin.rw
						&& yCoord >= 0 && yCoord < (int)thirdIntrin.rh) {
						coord[2 * (i*zw_ + j)] = xCoord;
						coord[2 * (i*zw_ + j) + 1] = yCoord;
					}
				}


			}
		}
		const auto nInter = hn_;
		for (int i = 0; i < zh_ - 1; i++) {
			for (int j = 0; j < zw_ - 1; j++) {
				auto top_left = &coord[2 * (i*zw_ + j)];
				auto top_rght = &coord[2 * (i*zw_ + j + 1)];
				auto bot_left = &coord[2 * ((i + 1)*zw_ + j)];
				auto val = zD[i*zw_ + j];
				int valR = zD[i*zw_ + j + 1];
				int valD = zD[(i + 1)*zw_ + j];


				if (*top_left) {
					auto xMin = *top_left;
					auto yMin = *(top_left + 1);
					auto xMax = (*(top_rght) && *(top_rght)-xMin < nInter) ? *(top_rght) : xMin + 1;
					auto yMax = (*(bot_left + 1) && *(bot_left + 1) - yMin < nInter) ? *(bot_left + 1) : yMin + 1;
					for (int y = yMin; y < yMax; y++) {
						for (int x = xMin; x < xMax; x++) {
							const auto v = zOut[y*rgb_width + x];
							zOut[y*rgb_width + x] = (v != 0 && v < val) ? v : val;
						}
					}
				}
			}
		}
	}
	void getRGBAlignedHF(const uint16_t* zIn, uint16_t* zOut)
	{
		memset(coord.get(), 0, sizeof(uint16_t) * 2 * zw_*zh_);
		const auto rgb_width = ds->accessThird()->thirdWidth();
		const auto rgb_height = ds->accessThird()->thirdHeight();
		const uint16_t * zD = zIn;
		DSCalibIntrinsicsRectified zIntrin, thirdIntrin;
		double zToThirdTrans[3];
		ds->getCalibIntrinsicsZ(zIntrin);
		ds->accessThird()->getCalibIntrinsicsRectThird(thirdIntrin);
		ds->accessThird()->getCalibExtrinsicsZToRectThird(zToThirdTrans);


		for (int i = 0; i < zh_; i++) {
			for (int j = 0; j < zw_; j++) {
				if (zD[i*zw_ + j] != 0) {
					float zImage[] = { static_cast<float>(j), static_cast<float>(i), (float)zD[i*zw_ + j] }, zCamera[3], thirdCamera[3], thirdImage[2];
					DSTransformFromZImageToZCamera(zIntrin, zImage, zCamera);
					DSTransformFromZCameraToRectThirdCamera(zToThirdTrans, zCamera, thirdCamera);
					DSTransformFromThirdCameraToRectThirdImage(thirdIntrin, thirdCamera, thirdImage);
					int yCoord = static_cast<int>(thirdImage[1] + 0.5);
					int xCoord = static_cast<int>(thirdImage[0] + 0.5);
					if (xCoord >= 0 && xCoord < (int)thirdIntrin.rw
						&& yCoord >= 0 && yCoord < (int)thirdIntrin.rh) {
						zOut[yCoord*rgb_width + xCoord] = zD[i*zw_ + j];
					}
				}


			}
		}
		for (int y = 0; y < rgb_height; y++) {
			auto outputDepthRow = &zOut[y*rgb_width];
			for (int x = 1; x < rgb_width - 1; x++) {
				const auto v1 = outputDepthRow[x - 1];
				const auto v2 = outputDepthRow[x];
				const auto v3 = outputDepthRow[x + 1];
				if (v1 && !v2 && v3) {
					outputDepthRow[x] = (v1 + v3) >> 1;
				}
			}
		}
		for (int y = 1; y < rgb_height - 1; y++) {
			auto outputDepthRowP = &zOut[(y - 1)*rgb_width];
			auto outputDepthRowC = &zOut[y*rgb_width];
			auto outputDepthRowN = &zOut[(y + 1)*rgb_width];
			for (int x = 0; x < rgb_width; x++) {
				const auto v1 = outputDepthRowP[x];
				const auto v2 = outputDepthRowC[x];
				const auto v3 = outputDepthRowN[x];
				if (v1 && !v2 && v3) {
					outputDepthRowC[x] = (v1 + v3) >> 1;
				}
			}
		}
	}
private:
	std::unique_ptr<uint16_t[]> coord;
	int zw_, zh_, hn_;
	DSAPI * ds;
};
