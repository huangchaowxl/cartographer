#include "cartographer/mapping_2d/location.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping_2d/ray_casting.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "glog/logging.h"
#include <vector>
namespace cartographer {
namespace mapping_2d {


    location::location()
    {

    }
   void location::pure_location()

	{

	}
   std::vector<double> location::get_location()
    {

        return my_location;
    }

}  // namespace mapping_2d
}  // namespace cartographer

