/*
 * fft.c
 *
 *
 *
 */

/*FFT.C-FFT RADIX-2 USING DIF. FOR UP TO 128 POINTS */
#include "twid256.h" /*header file with twiddle constants*/
#include "fft.h"


void FFT(struct cmpx *Y, int N) /*FFT de vector de N puntos*/
{
	struct cmpx temp1, temp2; /*temporary storage variables */
	int i, j, k; /*loop counter variables */
	int upper_leg, lower_leg; /*index of upper/lower butterfly leg*/
	int leg_diff; /*difference between upper/lower leg*/
	int num_stages = 0; /*number of FFT stages, or iterations*/
	int index, step; /*index and step between twiddle factor*/
	/* log(base 2) de N puntos = M fases o estados */
	i = 1;
	do {
		num_stages += 1;
		i = i * 2;
	} while (i != N);
	/* starting difference between upper and lower butterfly legs*/
	leg_diff = N / 2;
	/* step between values in twiddle factor array twid256.h */
	step = 512 / N;
	/* For N-point FFT */
	for (i = 0; i < num_stages; i++) /*tantas iteraciones como fases*/
	{
		index = 0;
		for (j = 0; j < leg_diff; j++) /*tantas iteraciones como transfor-*/
		{ /*madas en cada fase*/
			for (upper_leg = j; upper_leg < N; upper_leg += (2 * leg_diff)) { /*tantas iteraciones como puntos en cada transformada*/
				lower_leg = upper_leg + leg_diff;
				temp1.real = (Y[upper_leg]).real + (Y[lower_leg]).real;
				temp1.imag = (Y[upper_leg]).imag + (Y[lower_leg]).imag;
				temp2.real = (Y[upper_leg]).real - (Y[lower_leg]).real;
				temp2.imag = (Y[upper_leg]).imag - (Y[lower_leg]).imag;
				(Y[lower_leg]).real = temp2.real * (w[index]).real
						- temp2.imag * (w[index]).imag;
				(Y[lower_leg]).imag = temp2.real * (w[index]).imag
						+ temp2.imag * (w[index]).real;
				(Y[upper_leg]).real = temp1.real;
				(Y[upper_leg]).imag = temp1.imag;
			}
			index += step;
		}
		leg_diff = leg_diff / 2;
		step *= 2;
	}
	/* bit reversal for resequencing data */
	j = 0;
	for (i = 1; i < (N - 1); i++) {
		k = N / 2;
		while (k <= j) {
			j = j - k;
			k = k / 2;
		}
		j = j + k;
		if (i < j) {
			temp1.real = (Y[j]).real;
			temp1.imag = (Y[j]).imag;
			(Y[j]).real = (Y[i]).real;
			(Y[j]).imag = (Y[i]).imag;
			(Y[i]).real = temp1.real;
			(Y[i]).imag = temp1.imag;
		}
	}

	return;
}

