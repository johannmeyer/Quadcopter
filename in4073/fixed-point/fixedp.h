/*Written by Konstantinos-P. Metaxas*/
#ifndef FIXEDP_H

#define FIXEDP_H

#include <stdint.h>

typedef int32_t fp;

#define FPQ(n)	((int)n)

//The fixed-point representation of constants can be computed during compilation to save CPU time.
#define FPCONST(num, q) ( (fp) (((double)num) * (1<<(int) q) >=0) ? ((double)num) * (1<<(int) q) + 0.5 : ((double)num) * (1<<(int) q) - 0.5 )

inline fp int2fp(int num, uint8_t q) {	return num << q; };
inline fp double2fp(double num, uint8_t q) { return (fp)(num * (1 << q)); };
inline fp float2fp(float num, uint8_t q) {	return (fp)(num * (1 << q)); };

fp int2fpDO(int num, uint8_t q);
fp double2fpDO(double num, uint8_t q);
fp float2fpDO(float num, uint8_t q);

inline int fp2int(fp num, uint8_t q) {	return (num + (1 << (q - 1))) >> q; };
inline double fp2double(fp num, uint8_t q) { return ((double)num) / (1 << q); };
inline float fp2float(fp num, uint8_t q) {	return ((float)num) / (1 << q); };

inline fp fpchangeQ(fp num, uint8_t initQ, uint8_t desQ)
{
	register int8_t diffQ = initQ - desQ;
	return ((diffQ >= 0) ? num >> diffQ : num << (-diffQ));
};
fp fpchangeQDO(fp num, uint8_t initQ, uint8_t desQ);

fp fpmult(fp num1, fp num2, uint8_t q);
fp fpmultDetQ(fp num1, fp num2, uint8_t initQ, uint8_t desQ);
fp fpaddDO(fp num1, fp num2, uint8_t q);
fp fpsubDO(fp num1, fp num2, uint8_t q);
fp fpmultDO(fp num1, fp num2, uint8_t q);
fp fpmultDetQDO(fp num1, fp num2, uint8_t initQ, uint8_t desQ);


#endif //FIXEDP_H
