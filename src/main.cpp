#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "systemA\sysA.h"
#include "systemB\sysB.h"
#include <mathF.h>

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		fprintf(stdout, "Usage: %s number\n", argv[0]);
		return 1;
	}

	sysAadd(1, 1);
	sysBadd(1, 1);

	double inputValue = atof(argv[1]);
	double outputValue = sqrt(inputValue);
	fprintf(stdout, "The square root of %g is %g\n",
		inputValue, outputValue);
	return 0;
}