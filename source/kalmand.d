module kalmand;

import diffengine :
    Parameter,
    Differentiable,
    constant,
    square;

@safe:

struct KalmanFilter(F, P)
{
    struct Parameters
    {
        P drift;
        P tension;
        P cons;
        P measureVariance;
        P stateVariance;
    }

    @disable this();

    this(Parameters parameters, F initState, F initVariance, F one) nothrow pure scope
    {
        this.params_ = parameters;
        this.state_ = initState;
        this.variance_ = initVariance;
        this.one_ = one;
    }

    F estimate(F x) nothrow pure return scope
    {
        estimateState_ = params_.drift + params_.tension * state_;
        estimateMeasure_ = params_.cons + estimateState_ * x;
        estimateVariance_ = params_.tension.square * variance_ + params_.stateVariance;
        return estimateMeasure_;
    }

    F filtering(F x, F y) nothrow pure scope
    {
        auto k = (x * estimateVariance_)
            / (x.square * estimateVariance_ + params_.measureVariance);
        state_ = estimateState_ + k * (y - estimateMeasure_);
        variance_ = (one_ - x * k) * estimateVariance_;
        return state_;
    }

private:
    Parameters params_;
    F one_;
    F state_;
    F variance_;

    F estimateState_;
    F estimateVariance_;
    F estimateMeasure_;
}

