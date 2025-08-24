#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>
#include <numeric>  // std::accumulate

ignition::transport::Node node;
ignition::transport::Node::Publisher pubScan;
ignition::transport::Node::Publisher pubLinear;

void cb(const ignition::msgs::LaserScan &_msg)
{
  // Publica o scan completo
  pubScan.Publish(_msg);

  // Calcula a média das distâncias válidas
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < _msg.ranges_size(); ++i)
  {
    double val = _msg.ranges(i);
    if (std::isfinite(val)) {
      sum += val;
      ++count;
    }
  }

  double avg = (count > 0) ? sum / count : 0.0;
  ignition::msgs::Double avgMsg;
  avgMsg.set_data(avg);
  pubLinear.Publish(avgMsg);
}

int main(int argc, char **argv)
{
  pubScan = node.Advertise<ignition::msgs::LaserScan>("/scan");
  pubLinear = node.Advertise<ignition::msgs::Double>("/scan/linear");

  if (!node.Subscribe("/lidar", cb))
  {
    std::cerr << "Erro ao se inscrever em /lidar" << std::endl;
    return -1;
  }

  ignition::transport::waitForShutdown();
  return 0;
}

