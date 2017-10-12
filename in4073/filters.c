#include "filters.h"
#include "sensors.h"


fp butter(int16_t newSample, uint8_t sensorId)
{
	const fp a1 = FPCONST(-1.375, FPQ(10));
	const fp a2 = FPCONST(0.53125, FPQ(10));
	static fp xphi[3], yphi[3], xtheta[3], ytheta[3], xpsi[3], ypsi[3];

	if (sensorId == PHI)
	{
		xphi[2] = xphi[1]; xphi[1] = xphi[0];
		yphi[2] = yphi[1]; yphi[1] = yphi[0];

		xphi[0] = newSample;

		yphi[0] = (xphi[0] + (xphi[1] << 1) + xphi[2]) - fpmult(a1, yphi[1], FPQ(10)) - fpmult(a2, yphi[2], FPQ(10));

		return yphi[0] << 5;
	}
	if(sensorId == THETA)
	{
		xtheta[2] = xtheta[1]; xtheta[1] = xtheta[0];
		ytheta[2] = ytheta[1]; ytheta[1] = ytheta[0];

		xtheta[0] = newSample;

		ytheta[0] = (xtheta[0] + (xtheta[1] << 1) + xtheta[2]) - fpmult(a1, ytheta[1], FPQ(10)) - fpmult(a2, ytheta[2], FPQ(10));

		return ytheta[0]<<5;
	}
	if (sensorId == PSI)
	{
		xpsi[2] = xpsi[1]; xpsi[1] = xpsi[0];
		ypsi[2] = ypsi[1]; ypsi[1] = ypsi[0];

		xpsi[0] = newSample;

		ypsi[0] = (xpsi[0] + (xpsi[1] << 1) + xpsi[2]) - fpmult(a1, ypsi[1], FPQ(10)) - fpmult(a2, ypsi[2], FPQ(10));

		return ypsi[0] << 5;
	}
	return 0;
}
