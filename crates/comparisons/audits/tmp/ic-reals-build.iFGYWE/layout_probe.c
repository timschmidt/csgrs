#include <stdio.h>
#include "real.h"

int main(void)
{
	printf("Real=%zu Tag=%zu Vec=%zu MatX=%zu TenXY=%zu DigsX=%zu SignX=%zu Cls=%zu Alt=%zu PredX=%zu BoolX=%zu BoolXY=%zu\n",
		sizeof(Real), sizeof(Tag), sizeof(Vec), sizeof(MatX), sizeof(TenXY),
		sizeof(DigsX), sizeof(SignX), sizeof(Cls), sizeof(Alt), sizeof(PredX),
		sizeof(BoolX), sizeof(BoolXY));
	return 0;
}
