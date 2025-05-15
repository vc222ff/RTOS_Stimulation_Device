// Checks that evaluation.h header is not defined. Defines it.
#ifndef EVALUATION_H_
#define EVALUATION_H_

// Evaluation results structure.
typedef struct {
    float rmse;
    float percent_within_range;
    int total;
    int within_range;
} EvaluationResult;

#endif
