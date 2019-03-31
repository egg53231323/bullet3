#include "AnimationUtility.h"
#include <limits>

namespace AnimationUtility
{
	const AnimationKeyTime Invalid_Time = std::numeric_limits<AnimationKeyTime>::lowest();

	template <typename T>
	T defaultInterpolateFunc(const AnimationKey<T> &a, const AnimationKey<T> &b, const AnimationKeyTime &targetTime)
	{
		if (targetTime <= a.time || a.time > b.time)
		{
			return a.value;
		}
		if (targetTime >= b.time)
		{
			return b.value;
		}
		return T(a.value + (targetTime - a.time) / (b.time - a.time) * (b.value - a.value));
	}

	float interpolateFuncFloat(const AnimationKey<float> &a, const AnimationKey<float> &b, const AnimationKeyTime &targetTime)
	{
		return defaultInterpolateFunc<float>(a, b, targetTime);
	}

	double interpolateFuncDouble(const AnimationKey<double> &a, const AnimationKey<double> &b, const AnimationKeyTime &targetTime)
	{
		return defaultInterpolateFunc<double>(a, b, targetTime);
	}
}