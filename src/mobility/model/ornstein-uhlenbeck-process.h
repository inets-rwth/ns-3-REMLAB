#ifndef ORNSTEIN_UHLENBECK_PROCESS_H
#define ORNSTEIN_UHLENBECK_PROCESS_H

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/random-variable-stream.h"
#include <cmath>
#include <vector>

namespace ns3 {

struct OrnsteinUhlenbeckErrorTraceParams
{
  double m_theta{0.}; // mean reversion rate
  double m_mu{0.}; // long-term average
  double m_sigma{0.}; // volatility

  double m_deltaT{0.}; // time delte between iterations
  double m_deltaW{0.}; // normal distribution with mean 0 and std deviation m_deltaT
  double m_Xprev{0.}; // previous value of the variable
  double m_Xnext{0.}; // next value of the variable

  /// m_theta * (m_mu - m_X) * m_deltaT
  double m_restoringTerm{0.}; // first term - restoring term suppresses fluctuations. x-dependent. The larger the deviation, the stronger the suppression
  /// m_sigma * m_deltaW
  double m_stochasticTerm{0.}; // second term - stochatsic terms generates fluctuations. x-independent
};

class OrnsteinUhlenbeckErrorModel : public Object
{
public:
  OrnsteinUhlenbeckErrorModel(double theta, double mu, double sigma);
  OrnsteinUhlenbeckErrorModel() = default;

  /// Assign stream to the underlying random number generator
  void AssignStreams(int64_t stream);

  double GetNextValue();

  void Reset(double initialValue);

  static TypeId GetTypeId(void);

  typedef void (*InternalStateTracedCallback)
      (const OrnsteinUhlenbeckErrorTraceParams internalState);

private:
  double m_theta; // Rate of mean reversion
  double m_mu;    // Long-term mean
  double m_sigma; // Volatility
  double m_lastInvocationTime{0.}; // Time increment
  double m_X{0.};     // Current state
  Ptr<NormalRandomVariable> m_generator;
  TracedCallback<OrnsteinUhlenbeckErrorTraceParams> m_varStateTrace;
};

} // namespace ns3

#endif /* ORNSTEIN_UHLENBECK_PROCESS_H */