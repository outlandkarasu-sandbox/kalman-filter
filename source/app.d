import karasux.random : gaussianDistributionRandom;

struct StateSpaceModel(T = real)
{
    T opCall()(scope auto ref const(T) x) @safe
    {
        state = drift + trend * state + random(stateVariance);
        return constant + state * x + random(errorVariance);
    }

    T drift;
    T trend;
    T constant;
    T errorVariance;
    T stateVariance;
    T state;

private:

    T random(scope ref const(T) variance) const @safe scope
    {
        return gaussianDistributionRandom!T() * variance;
    }
}

void main()
{
    import std.stdio : writeln;
    import std.range : generate, take;

    StateSpaceModel!() model = {
        drift: 0.0,
        trend: 1.0,
        constant: 0.0,
        errorVariance: 1.0,
        stateVariance: 1.0,
        state: 0.0,
    };

    generate!(() => model(1.0)).take(10).writeln;
}

