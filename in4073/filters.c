/*Written by Konstantinos-P. Metaxas*/
#include "filters.h"
#include "sensors.h"

//Low-pass 2nd order Butterworth filter with F/Fs= 0.1
//b0=0.03125, b1=0.0625, b2=0.03125
//a0=1, a1=-1.375, a2=0.53125
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

//Kalman filter with C1=16, C2=524288, p2phi=0.109375.

fp Kalman(fp sOrient, int16_t sRate, uint8_t sensorId)
{
	static fp phiRate = 0, thetaRate = 0;
	static fp phiBias20 = 0, thetaBias20 = 0;
	static fp phiOrient = 0, thetaOrient = 0;
	static fp phiFpsRate = 0;
	static fp thetaFpsRate = 0;
	fp error = 0;

	if (sensorId == PHI)
	{
		phiRate = phiFpsRate - fpchangeQ(phiBias20, FPQ(20), FPQ(10));

		phiOrient = phiOrient + (phiRate >> 3) - (phiRate >> 6);

		error = phiOrient - sOrient;

		phiOrient = phiOrient - (error >> 4);

		phiBias20 = phiBias20 + (error >> 6) + (error >> 9);

		phiFpsRate = int2fp(sRate, FPQ(10));

		return phiOrient;
	}
	else
	{
		thetaRate = thetaFpsRate - fpchangeQ(thetaBias20, FPQ(20), FPQ(10));

		thetaOrient = thetaOrient - ((thetaRate >> 3) - (thetaRate >> 6));

		error = thetaOrient- sOrient;

		thetaOrient = thetaOrient - (error >> 4);

		thetaBias20 = thetaBias20 + (error >> 6) + (error >> 9);

		thetaFpsRate = int2fp(sRate, FPQ(10));

		return thetaOrient;
	}
}
