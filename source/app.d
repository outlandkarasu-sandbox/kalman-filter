import std.stdio;

struct KalmanFilter(T = real)
{
    T opCall(scope auto ref const(T) x) nothrow pure @safe
    {
        state_ = drift_ + trend_ * state_ + error(stateVarience_);
        return constant_ + state_ * x + error(errorVarience_);
    }

private:
    T drift_;
    T trend_;
    T constant_;
    T errorVariance_;
    T stateVarience_;
    T state_;
}

void main()
{
	writeln("Edit source/app.d to start your project.");
}
