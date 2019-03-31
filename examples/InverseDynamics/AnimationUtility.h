#ifndef ANIMATION_UTILITY_H
#define ANIMATION_UTILITY_H

#include <vector>

typedef __int64 AnimationKeyTime;
template<typename T>
class AnimationKey
{
public:
	AnimationKey() : time(0), value()
	{
	}
	AnimationKeyTime time;
	T value;
};

template <typename T>
class AnimationCurve
{
public:
	typedef T (*interpolateFuncPtr)(const AnimationKey<T> &a, const AnimationKey<T> &b, const AnimationKeyTime &targetTime);

	AnimationCurve() : keyInterpolateFunc(NULL), lastIdx(0) {}
	AnimationCurve(interpolateFuncPtr func) : keyInterpolateFunc(func), lastIdx(0){}

	T getValue(const AnimationKeyTime &time) const
	{
		int count = (int)keys.size();
		if (NULL == keyInterpolateFunc || count <= 0)
		{
			return T();
		}
		if (lastIdx > count)
		{
			lastIdx = 0;
		}

		int searchIdx = 0;
		if (time >= keys[lastIdx].time)
		{
			searchIdx = lastIdx;
		}
		while (searchIdx < count && time > keys[searchIdx].time)
		{
			searchIdx++;
		}

		int idxA = searchIdx - 1, idxB = searchIdx;
		lastIdx = idxA;
		if (searchIdx >= count)
		{
			lastIdx = idxA = idxB = count - 1;
		}
		else if (searchIdx == 0)
		{
			lastIdx = idxA = idxB = 0;
		}

		return keyInterpolateFunc(keys[idxA], keys[idxB], time);
	}

	std::vector<AnimationKey<T> > keys;

private:
	interpolateFuncPtr keyInterpolateFunc;
	mutable int lastIdx;
};

namespace AnimationUtility
{
	extern const AnimationKeyTime Invalid_Time;
	float interpolateFuncFloat(const AnimationKey<float> &a, const AnimationKey<float> &b, const AnimationKeyTime &targetTime);
	double interpolateFuncDouble(const AnimationKey<double> &a, const AnimationKey<double> &b, const AnimationKeyTime &targetTime);
}

AnimationCurve<float>::AnimationCurve() : keyInterpolateFunc(AnimationUtility::interpolateFuncFloat), lastIdx(0) {}
AnimationCurve<double>::AnimationCurve() : keyInterpolateFunc(AnimationUtility::interpolateFuncDouble), lastIdx(0) {}

#endif  //ANIMATION_UTILITY_H
