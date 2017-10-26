/*Written by Konstantinos-P. Metaxas*/

#include "fixedp.h"
#include <stdio.h>

#define OVERFLOW_DETECT 0x80000000

//Integer to fixed-point decimal conversion to be called during design phase for
//overflow detection purposes.
fp int2fpDO(int num, uint8_t q)
{
	//if the number is larger than the integer part of the desired Q-precision,
	//overflow is detected.
	if (((num >= 0) ? num : -num) > (1 << (31 - q)) - 1)
	{
		printf("Overflow detected when converting %d to Q%d fixed-point\n", num, q);
		return 0;
	}
	else return num << q;
}

//Double presicion floating to fixed-point decimal conversion to be called during
//design phase for overflow detection purposes.
fp double2fpDO(double num, uint8_t q)
{
	//if the integer part of the number is larger than the integer part of the desired Q-precision,
	//overflow is detected.
	if (((num >= 0) ? num : -num) > (1 << (31 - q)) - 1)
	{
		printf("Overflow detected when converting %f to Q%d fixed-point\n", num, q);
		return 0;
	}
	else return (fp)(num * (1 << q));;
}


//Floating to fixed-point decimal conversion to be called during design phase for
//overflow detection purposes.
fp float2fpDO(float num, uint8_t q)
{
	//if the integer part of the number is larger than the integer part of the desired Q-precision,
	//overflow is detected.
	if (((num >= 0) ? num : -num) > (2 << (31 - q)) - 1)
	{
		printf("Overflow detected when converting %f to Q%d fixed-point\n", num, q);
		return 0;
	}
	else return (fp)(num * (1 << q));;
}


//Fixed-point with initQ precision conversion to fixed-point with desQ presicion
//to be called during design phase for overflow detection purposes.
fp fpchangeQDO(fp num, uint8_t initQ, uint8_t desQ)
{
	register int8_t diffQ = initQ - desQ;

	if (diffQ >= 0) return num >> diffQ;
	else
	{
		//if the integer part of the number is larger than the integer part of the desired Q-precision,
		//overflow is detected.
		double dnum = fp2double(num, initQ);
		if ((dnum >=0 ) ? dnum : -dnum > ((1 << (31 - desQ)) - 1))
		{
			printf("Overflow detected when trying to convert %f to Q%d\n", dnum, desQ);
			return 0;
		}
		else return num << (-diffQ);
	}
}

//Fixed-point decimal multiplication.
fp fpmult(fp num1, fp num2, uint8_t q)
{
	return (num1 * num2) >> q;
}

//Fixed-point decimal multiplication with change of presicion.
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

//Fixed-point decimals addition to be called during design phase for
//overflow detection purposes.
fp fpaddDO(fp num1, fp num2, uint8_t q)
{
	register uint32_t unum1 = num1;
	register uint32_t unum2 = num2;

	register uint32_t sum = unum1 + unum2;

	//if sign of the operands is the same but different than the sign of the result,
	//overflow is detected.
	if ((~(unum1 ^ unum2) & OVERFLOW_DETECT) && ((unum1 ^ sum) & OVERFLOW_DETECT))
	{
		printf("Overflow detected when adding %f to %f using Q%d presicion\n", fp2double(num1, q), fp2double(num2, q), q);
		return 0;
	}
	else return sum;
}

//Fixed-point decimal subtraction to be called during design phase for
//overflow detection purposes.
fp fpsubDO(fp num1, fp num2, uint8_t q)
{
	register uint32_t unum1 = num1;
	register uint32_t unum2 = num2;

	register uint32_t diff = unum1 - unum2;

	//if the sign of the operands is different and the sign of the result is different than
	//the sign of the minuend, overflow is detected.
	if (((unum1 ^ unum2) & OVERFLOW_DETECT) && ((unum1 ^ diff) & OVERFLOW_DETECT))
	{
		printf("Overflow detected when subtracting %f from %f using Q%d presicion\n", fp2double(num2, q), fp2double(num1, q), q);
		return 0;
	}
	else return diff;
}

//Fixed-point decimal multiplication to be called during design phase for
//overflow detection purposes.
fp fpmultDO(fp num1, fp num2, uint8_t q)
{
	register double dnum1 = (num1 >= 0) ? fp2double(num1, q) : -fp2double(num1, q);
	register double dnum2 = (num2 >= 0) ? fp2double(num2, q) : -fp2double(num2, q);

	//if the decimal fraction is less than 16, then after multiplication there are
	//bits remaining for the integer part.
	if (q < 16)
	{
		//if the product of the integer parts does not fit in the bits remaining to
		//represent the integer part after multiplication, then overflow is detected. 
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

//Fixed-point decimal multiplication with presicion change to be called during
//design phase for overflow detection purposes.
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
