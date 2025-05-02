#ifndef arduino_ros2_control_TRACK_HPP
#define arduino_ros2_control_TRACK_HPP

#include <string>
#include <cmath>


class Track
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Track() = default;

    Track(const std::string &track_name, int counts_per_rev)
    {
      setup(track_name, counts_per_rev);
    }

    
    void setup(const std::string &track_name, int counts_per_rev)
    {
      name = track_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }



};


#endif // arduino_ros2_control_TRACK_HPP
