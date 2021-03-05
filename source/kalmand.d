module kalmand;

import lbfgsd.math : square, log;
import karasux.random : gaussianDistributionRandom;

struct StateSpaceModel(T)
{
    T opCall()(scope auto ref const(T) x) @safe scope
    {
        updateState();
        return measure(x);
    }

    T measure()(scope auto ref const(T) x) const @safe scope
    {
        return constant + lastState * x + random(errorVariance);
    }

    void updateState() @safe scope
    {
        lastState = estimateState + random(stateVariance);
    }

    const @nogc nothrow pure @safe scope
    {
        T estimateMeasurement()(scope auto ref const(T) x) 
        {
            return constant + estimateState() * x;
        }

        @property T estimateState()
        {
            return drift + trend * lastState;
        }
    }

    T drift;
    T trend;
    T constant;
    T errorVariance;
    T stateVariance;
    T lastState;

private:

    static T random(scope ref const(T) variance) @safe
    {
        return gaussianDistributionRandom!real() * variance;
    }
}

struct KalmanFilter(T)
{
    @nogc nothrow pure @safe scope
    {
        T estimateMeasurement()(auto scope ref const(T) x) const
        {
            return model.estimateMeasurement(x);
        }

        @property T estimateVariance() const
        {
            return square(model.trend) * variance + model.stateVariance;
        }

        T estimateError2()(auto scope ref const(T) x, auto scope ref const(T) y) const
        {
            return square(y - estimateMeasurement(x));
        }

        @property T estimateErrorVariance()(auto scope ref const(T) x) const
        {
            return square(x) * estimateVariance + model.errorVariance;
        }

        @property T estimateState() const
        {
            return model.estimateState;
        }

        T kalmanGain()(auto scope ref const(T) x) const
        {
            return x * estimateVariance / estimateErrorVariance(x);
        }

        void filtering()(auto scope ref const(T) x, auto scope ref const(T) y)
        {
            immutable k = kalmanGain(x);
            model.lastState = estimateState + k * (y - estimateMeasurement(x));
            variance = (cast(real) 1.0 - x * k) * estimateVariance;
        }
    }

    T likelihood()(auto scope ref const(T) x, auto scope ref const(T) y) @nogc nothrow pure @safe scope
    {
        immutable v = estimateErrorVariance(x);
        return log(v) + estimateError2(x, y) / v;
    }

    StateSpaceModel!T model;
    T variance;
}

