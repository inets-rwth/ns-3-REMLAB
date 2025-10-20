#include "ornstein-uhlenbeck-process.h"

namespace ns3 {

OrnsteinUhlenbeckErrorModel::OrnsteinUhlenbeckErrorModel(double theta, double mu, double sigma)
      : m_theta(theta), m_mu(mu), m_sigma(sigma)
{
  m_generator = CreateObject<NormalRandomVariable>();
  m_generator->SetAttribute("Mean", DoubleValue(0.0)); // TODO: expose to the outer world
  m_generator->SetAttribute("Variance", DoubleValue(0.25)); // TODO: expose to the outer world
  m_X = m_mu; // Initialize to the mean value.
}

TypeId
OrnsteinUhlenbeckErrorModel::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OrnsteinUhlenbeckErrorModel")
    .SetParent<Object>()
    .SetGroupName("Mobility")
    .AddConstructor<OrnsteinUhlenbeckErrorModel>()
    .AddTraceSource("InternalStateTrace",
                    "State of all internal variables",
                    MakeTraceSourceAccessor(&OrnsteinUhlenbeckErrorModel::m_varStateTrace),
                    "ns3::OrnsteinUhlenbeckErrorModel::InternalStateTracedCallback");
  return tid;
}

void
OrnsteinUhlenbeckErrorModel::AssignStreams(int64_t stream)
{
  m_generator->SetStream(stream);
}

double OrnsteinUhlenbeckErrorModel::GetNextValue()
{
  // double m_deltaT = Simulator::Now().GetSeconds() - m_lastInvocationTime;
  // m_lastInvocationTime = Simulator::Now().GetSeconds();
  // if (m_deltaT > 1)
  //   m_deltaT = 1; 
  double m_deltaT = 0.25;
  // std::cout << "OrnsteinUhlenbeckErrorModel::GetNextValue: time diff is " << m_deltaT << std::endl;
  // std::cout << "OrnsteinUhlenbeckErrorModel::GetNextValue: m_X is " << m_X << std::endl;

  double deltaW = sqrt(m_deltaT) * m_generator->GetValue(); // Wiener process increment
  // std::cout << "  * deltaW is " << deltaW << std::endl;
  // std::cout << "  * m_theta * (m_mu - m_X) * m_deltaT is " << m_theta * (m_mu - m_X) * m_deltaT << std::endl;
  // std::cout << "  * m_sigma * deltaW is " << m_sigma * deltaW << std::endl;

  OrnsteinUhlenbeckErrorTraceParams params;
  params.m_deltaT = m_deltaT;
  params.m_deltaW = deltaW;
  params.m_mu = m_mu;
  params.m_theta = m_theta;
  params.m_sigma = m_sigma;
  params.m_Xprev = m_X;

  // m_theta - rate of mean reversion
  // m_mu - long-term average
  // m_X - current value of the variable
  // deltaW - normal distribution with mean at 0 and variance at set to the time delta between invocations

  // first term - restoring term suppresses fluctuations. x-dependent. The larger the deviation, the stronger the suppression
  // second term - stochatsic terms generates fluctuations. x-independent
  // see https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10632843 formula 6
  m_X = m_X + m_theta * (m_mu - m_X) * m_deltaT + m_sigma * deltaW;
  // std::cout << "OrnsteinUhlenbeckErrorModel::GetNextValue: next value is " << m_X << std::endl << std::endl;
  
  params.m_Xnext = m_X;
  params.m_restoringTerm = m_theta * (m_mu - m_X) * m_deltaT;
  params.m_stochasticTerm = m_sigma * deltaW;
  m_varStateTrace(params);

  return m_X;
}

void OrnsteinUhlenbeckErrorModel::Reset(double initialValue)
{
  m_X = initialValue;
  // std::cout << "OrnsteinUhlenbeckErrorModel::Reset: m_X is set to " << initialValue << std::endl;
}

} // namespace ns3