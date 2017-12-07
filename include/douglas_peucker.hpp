#ifndef _DOUGLAS_PEUCKER_H_
#define _DOUGLAS_PEUCKER_H_

#include <Eigen/Eigen>
#include <vector>

/**
 * This class is used to simpified a curve containing numerous points,applying the
 * Ramer-Douglas-Peucker Algorithm.
 *
 * Usage : Create a instance of this class, then call the simplifiy() function
 * Input : Points of a curve, and epsilon
 * Output: Points of the simplified curve
 */
class RDPCurveSimplifier
{
public:
  RDPCurveSimplifier()
  {
  }

  void simplify(std::vector<Eigen::Vector3d> &curve, float epsilon);

private:
  void getMostDistantPoint(std::vector<Eigen::Vector3d> segment, int &point_id, float &distance);

  void breakSegment(std::vector<Eigen::Vector3d> segment, int point_id,
                    std::vector<Eigen::Vector3d> &part1, std::vector<Eigen::Vector3d> &part2);

  bool allSegmentSimplified(std::vector<std::vector<Eigen::Vector3d>> segments);

  void getDistanceToLine(Eigen::Vector3d front, Eigen::Vector3d back, Eigen::Vector3d point,
                         float &distance);
};

void RDPCurveSimplifier::simplify(std::vector<Eigen::Vector3d> &curve, float epsilon)
{
  // create a vector containing the simplified segments
  std::vector<std::vector<Eigen::Vector3d>> segments;

  // At the begining, there is only one segment
  segments.push_back(curve);

  // begin to simplify the curve, applying Ramer-Douglas-Peucker Algorithm
  while(1)
  {
    // output the segment
    std::cout << "size:" << std::endl;
    for(int i= 0; i < segments.size(); i++)
    {
      std::cout << segments[i].size() << ",";
    }
    std::cout << std::endl;

    std::vector<std::vector<Eigen::Vector3d>> updated_segments;

    // simplify each segment
    for(int i= 0; i < segments.size(); i++)
    {
      // get one segment
      std::vector<Eigen::Vector3d> sgm= segments[i];
      if(sgm.size() == 2)
      {
        updated_segments.push_back(sgm);
        continue;
      }

      // get the most distant point's id and distance in this segment
      float dist= -1.0;
      int pt_id= -1;
      getMostDistantPoint(sgm, pt_id, dist);
      // if distance larger than epsilon, break the segemnt into two parts
      if(dist > epsilon)
      {
        std::vector<Eigen::Vector3d> part1, part2;
        breakSegment(sgm, pt_id, part1, part2);
        updated_segments.push_back(part1);
        updated_segments.push_back(part2);
      }
      // distance small enough, only hold front and back point
      else
      {
        std::vector<Eigen::Vector3d> front_back;
        front_back.push_back(sgm.front());
        front_back.push_back(sgm.back());
        updated_segments.push_back(front_back);
      }
    }

    // after one iteration, reset the segments
    segments= updated_segments;
    updated_segments.clear();

    // if all segments are simplified, stop the iteration, else the iteration continue
    if(allSegmentSimplified(segments))
      break;
  }

  // finally reset points in curve
  curve.clear();
  for(int i= 0; i < segments.size(); i++)
  {
    curve.insert(curve.end(), segments[i].begin(), segments[i].end());
  }
}

void RDPCurveSimplifier::getMostDistantPoint(std::vector<Eigen::Vector3d> segment, int &point_id,
                                             float &distance)
{
  Eigen::Vector3d front= segment.front();
  Eigen::Vector3d back= segment.back();

  point_id= -1;
  distance= -10.0;
  for(int i= 1; i < segment.size() - 1; i++)
  {
    float dist= 0;
    getDistanceToLine(front, back, segment[i], dist);
    if(dist > distance)
    {
      distance= dist;
      point_id= i;
    }
  }
}

void RDPCurveSimplifier::breakSegment(std::vector<Eigen::Vector3d> segment, int point_id,
                                      std::vector<Eigen::Vector3d> &part1,
                                      std::vector<Eigen::Vector3d> &part2)
{
  part1.insert(part1.begin(), segment.begin(), segment.begin() + point_id + 1);
  part2.insert(part2.begin(), segment.begin() + point_id, segment.end());
}

bool RDPCurveSimplifier::allSegmentSimplified(std::vector<std::vector<Eigen::Vector3d>> segments)
{
  for(int i= 0; i < segments.size(); i++)
  {
    if(segments[i].size() != 2)
    {
      return false;
    }
  }

  return true;
}

void RDPCurveSimplifier::getDistanceToLine(Eigen::Vector3d front, Eigen::Vector3d back,
                                           Eigen::Vector3d point, float &distance)
{
  Eigen::Vector4d line_dir(back(0) - front(0), back(1) - front(1), back(2) - front(2), 0);
  line_dir.normalize();
  Eigen::Vector4d p2p(point(0) - front(0), point(1) - front(1), point(2) - front(2), 0);

  distance= p2p.cross3(line_dir).squaredNorm();
  distance= sqrt(distance);
}

#endif