/*
 * =====================================================================================
 *
 *    Description:  Corblivar mathematical stuff
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_MATH
#define _CORBLIVAR_MATH

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any

class Math {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	// private in order to avoid instances of ``static'' class
	private:
		Math() {
		}

	// public data, functions
	public:
		// random-number functions
		// note: range is [min, max)
		inline static int randI(int const& min, int const& max) {
			if (max == min) {
				return min;
			}
			else {
				return min + (rand() % (max - min));
			}
		};
		inline static bool randB() {

			int const r = rand();
			return (r < (RAND_MAX / 2));
		};
		inline static double randF01() {
			return ((double) rand() / RAND_MAX);
		}

		// standard deviation of samples
		inline static double stdDev(vector<double> const& samples) {
			double avg, sq_diffs;

			// determine avg of samples
			avg = 0.0;
			for (double const& s : samples) {
				avg += s;
			}
			avg /= samples.size();

			// determine sum of squared diffs for std dev
			sq_diffs = 0.0;
			for (double const& s : samples) {
				sq_diffs += pow(s - avg, 2.0);
			}

			// determine std dev
			return sqrt(sq_diffs / ((double) samples.size()));
		}

		// 1D gauss function; used for separated convolution w/ 2D gauss function,
		// provides the impulse response function for power blurring
		inline static double gauss1D(double const& value, double const& factor, double const& spread) {
			return factor * exp(-(1.0 / spread) * pow(value, 2.0));
		}
};

#endif
