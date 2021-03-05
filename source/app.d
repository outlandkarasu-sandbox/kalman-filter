import std.math : pow;

import karasux.random : gaussianDistributionRandom;

struct StateSpaceModel(T = real)
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
        return gaussianDistributionRandom!T() * variance;
    }
}

struct KalmanFilter(T = real)
{
    @nogc nothrow pure @safe scope
    {
        T estimateMeasurement()(auto scope ref const(T) x) const
        {
            return model.estimateMeasurement(x);
        }

        @property T estimateVariance() const
        {
            return pow2(model.trend) * variance + model.stateVariance;
        }

        @property T estimateState() const
        {
            return model.estimateState;
        }

        T kalmanGain()(auto scope ref const(T) x) const
        {
            immutable variance = estimateVariance;
            return variance * model.trend / (pow2(x) * variance + model.errorVariance);
        }

        void filtering()(auto scope ref const(T) x, auto scope ref const(T) y)
        {
            immutable k = kalmanGain(x);
            model.lastState = estimateState + k * (y - estimateMeasurement(x));
            variance = (cast(T) 1.0 - x * k) * variance;
        }
    }

    StateSpaceModel!T model;
    T variance;

private:

    static T pow2()(auto scope ref const(T) x) @nogc nothrow pure @safe
    {
        return pow(x, cast(T) 2.0);
    }
}

void main()
{
    import std.stdio : writefln;

    StateSpaceModel!() model = {
        drift: 0.0,
        trend: 1.0,
        constant: 0.0,
        errorVariance: 1.0,
        stateVariance: 1.0,
        lastState: 0.0,
    };
    KalmanFilter!() kalmanFilter = {
        model: model,
        variance: 10000.0,
    };

    foreach (i; 0 .. 100)
    {
        immutable real x = 1.0;
        immutable ey = kalmanFilter.estimateMeasurement(x);
        immutable y = model(x);
        immutable error = y - ey;
        kalmanFilter.filtering(x, y);
        writefln("x: %s, estimate y: %s, y: %s, error: %s, variance: %s",
            x, ey, y, error, kalmanFilter.variance);
    }
}

