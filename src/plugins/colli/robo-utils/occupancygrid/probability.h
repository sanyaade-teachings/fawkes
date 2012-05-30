#ifndef PROBABILITY_H
#define PROBABILITY_H

typedef float Probability;

inline bool isProb(Probability p)
{
  return ((p >= 0) && (p <= 1));
}

#endif
