#include "fixedp.h"
#include <stdio.h>

#define OVERFLOW_DETECT 0x80000000

fp int2fpDO(int num, uint8_t q)
{
	if (((num >= 0) ? num : -num) > (1 << (31 - q)) - 1)
	{
		printf("Overflow detected when converting %d to Q%d fixed-point\n", num, q);
		return 0;
	}
	else return num << q;
}

fp double2fpDO(double num, uint8_t q)
{
	if (((num >= 0) ? num : -num) > (1 << (31 - q)) - 1)
	{
		printf("Overflow detected when converting %f to Q%d fixed-point\n", num, q);
		return 0;
	}
	else return (fp)(num * (1 << q));;
}

fp float2fpDO(float num, uint8_t q)
{
	if (((num >= 0) ? num : -num) > (2 << (31 - q)) - 1)
	{
		printf("Overflow detected when converting %f to Q%d fixed-point\n", num, q);
		return 0;
	}
	else return (fp)(num * (1 << q));;
}

fp fpchangeQDO(fp num, uint8_t initQ, uint8_t desQ)
{
	register int8_t diffQ = initQ - desQ;

	if (diffQ >= 0) return num >> diffQ;
	else
	{
		double dnum = fp2double(num, initQ);
		if ((dnum >=0 ) ? dnum : -dnum > ((1 << (31 - desQ)) - 1))
		{
			printf("Overflow detected when trying to convert %f to Q%d\n", dnum, desQ);
			return 0;
		}
		else return num << (-diffQ);
	}
}

fp fpmult(fp num1, fp num2, uint8_t q)
{
	return (num1 * num2) >> q;
}

fp fpmultDetQ(fp num1, fp num2, uint8_t initQ, uint8_t desQ)
{
	register int16_t diffQ = initQ - desQ;

	if (diffQ < 0)
	{
		int8_t shiftDiffQ = (initQ << 1) - desQ;
		return (shiftDiffQ >= 0) ? (num1*num2) >> shiftDiffQ : (num1*num2) << -shiftDiffQ;
	}

	if (diffQ > 0) return ((num1 >> diffQ) * (num2 >> diffQ)) >> desQ;

	return (num1 * num2) >> desQ;
}

fp fpdiv(fp num1, fp num2, uint8_t q)
{
	return (num1 << q) / num2;
}



fp fpaddDO(fp num1, fp num2, uint8_t q)
{
	register uint32_t unum1 = num1;
	register uint32_t unum2 = num2;

	register uint32_t sum = unum1 + unum2;

	if ((~(unum1 ^ unum2) & OVERFLOW_DETECT) && ((unum1 ^ sum) & OVERFLOW_DETECT))
	{
		printf("Overflow detected when adding %f to %f using Q%d presicion\n", fp2double(num1, q), fp2double(num2, q), q);
		return 0;
	}
	else return sum;
}

fp fpsubDO(fp num1, fp num2, uint8_t q)
{
	register uint32_t unum1 = num1;
	register uint32_t unum2 = num2;

	register uint32_t diff = unum1 - unum2;

	if (((unum1 ^ unum2) & OVERFLOW_DETECT) && ((unum1 ^ diff) & OVERFLOW_DETECT))
	{
		printf("Overflow detected when subtracting %f from %f using Q%d presicion\n", fp2double(num2, q), fp2double(num1, q), q);
		return 0;
	}
	else return diff;
}

fp fpmultDO(fp num1, fp num2, uint8_t q)
{
	register double dnum1 = (num1 >= 0) ? fp2double(num1, q) : -fp2double(num1, q);
	register double dnum2 = (num2 >= 0) ? fp2double(num2, q) : -fp2double(num2, q);

	if (q < 16)
	{
		if (dnum1 * dnum2 > ((1 << (31 - (q << 1))) - 1))
		{
			printf("Overflow detected when multiplying %f with %f using Q%d presicion\n", dnum2, dnum1, q);
			return 0;
		}
		else return (num1*num2) >> q;
	}
	else
	{
		printf("Overflow detected when multiplying %f with %f using Q%d presicion\n", dnum2, dnum1, q);
		return 0;
	}
}

fp fpmultDetQDO(fp num1, fp num2, uint8_t initQ, uint8_t desQ)
{
	register int8_t diffQ = initQ - desQ;

	if (diffQ <= 0)
	{
		register double dnum1 = (num1 >= 0) ? fp2double(num1, initQ) : -fp2double(num1, initQ);
		register double dnum2 = (num2 >= 0) ? fp2double(num2, initQ) : -fp2double(num2, initQ);
		if (initQ < 16)
		{
			int multMaxVal = (1 << (31 - (initQ << 1))) - 1;
			int desQMaxVal = (1 << (31 - desQ)) - 1;
			int minMaxVal = (multMaxVal <= desQMaxVal) ? multMaxVal : desQMaxVal;
			if ((dnum1 * dnum2 > minMaxVal))
			{
				printf("Overflow detected when multiplying %f with %f using Q%d presicion. Max value allowed: %d\n", dnum1, dnum2, initQ, minMaxVal);
				return 0;
			}
			else
			{
				int8_t shiftDiffQ = (initQ << 1) - desQ;
				return (shiftDiffQ >= 0) ? (num1*num2) >> shiftDiffQ : (num1*num2) << -shiftDiffQ;
			}
		}
		else
		{
			printf("Overflow detected when multiplying %f with %f using Q%d presicion\n", fp2double(num2, initQ), fp2double(num1, initQ), initQ);
			return 0;
		}
	}
	else
	{
		register fp desNum1 = num1 >> diffQ;
		register fp desNum2 = num2 >> diffQ;
		register double dnum1 = fp2double(desNum1, desQ);
		register double dnum2 = fp2double(desNum2, desQ);

		if (desQ < 16)
		{
			int mval = (1 << (31 - (desQ << 1))) - 1;
			
			if (((num1 >= 0) ? dnum1 : -dnum1) * ((num2 >= 0) ? dnum2 : -dnum2) > mval)
			{
				printf("Overflow detected when multiplying %f with %f using Q%d presicion. Max value allowed: %d\n", dnum1, dnum2, desQ, mval);
				return 0;
			}
			else return (desNum1 * desNum2) >> desQ;
		}
		else
		{
			printf("Overflow detected when multiplying %f with %f using Q%d presicion\n", dnum1, dnum2, desQ);
			return 0;
		}
	}
}