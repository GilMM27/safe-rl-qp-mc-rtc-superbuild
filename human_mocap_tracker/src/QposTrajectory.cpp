#include "human_mocap_tracker/HumanMocapTracker.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace human_mocap
{

namespace
{
std::vector<double> parseLine(const std::string & line)
{
  std::vector<double> values;
  std::stringstream stream(line);
  std::string field;
  while(std::getline(stream, field, ','))
  {
    if(!field.empty())
    {
      values.push_back(std::stod(field));
    }
  }
  return values;
}
} // namespace

QposTrajectory::QposTrajectory(std::vector<TrajectoryFrame> frames) : frames_(std::move(frames))
{
  if(frames_.empty())
  {
    throw std::invalid_argument("qpos trajectory must contain at least one frame");
  }
  for(std::size_t i = 1; i < frames_.size(); ++i)
  {
    if(frames_[i].time <= frames_[i - 1].time || frames_[i].qpos.size() != frames_[0].qpos.size())
    {
      throw std::invalid_argument("qpos trajectory times must increase and sizes must match");
    }
  }
}

QposTrajectory QposTrajectory::loadCsv(const std::string & path, std::size_t qpos_size)
{
  std::ifstream input(path);
  if(!input)
  {
    throw std::runtime_error("cannot open qpos trajectory: " + path);
  }
  std::vector<TrajectoryFrame> frames;
  std::string line;
  while(std::getline(input, line))
  {
    if(line.empty() || line[0] == '#')
    {
      continue;
    }
    const auto values = parseLine(line);
    if(values.size() != qpos_size + 1)
    {
      throw std::invalid_argument("trajectory row must contain time plus qpos values");
    }
    frames.push_back({values.front(), {values.begin() + 1, values.end()}});
  }
  return QposTrajectory(std::move(frames));
}

TrajectoryFrame QposTrajectory::sample(double time, bool loop) const
{
  if(loop && duration() > 0.0)
  {
    time = std::fmod(std::max(0.0, time), duration());
  }
  time = std::clamp(time, 0.0, duration());
  const auto upper = std::upper_bound(
      frames_.begin(), frames_.end(), time, [](double value, const TrajectoryFrame & frame) { return value < frame.time; });
  if(upper == frames_.begin())
  {
    return frames_.front();
  }
  if(upper == frames_.end())
  {
    return frames_.back();
  }
  const auto & a = *(upper - 1);
  const auto & b = *upper;
  const double alpha = (time - a.time) / (b.time - a.time);
  TrajectoryFrame result{time, a.qpos};
  for(std::size_t i = 0; i < result.qpos.size(); ++i)
  {
    result.qpos[i] += alpha * (b.qpos[i] - a.qpos[i]);
  }
  return result;
}

double QposTrajectory::duration() const noexcept
{
  return frames_.back().time;
}

std::size_t QposTrajectory::qposSize() const noexcept
{
  return frames_.front().qpos.size();
}

} // namespace human_mocap
