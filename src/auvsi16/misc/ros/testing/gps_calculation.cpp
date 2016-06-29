// Example of using the GeographicLib::LocalCartesian class
#include <iostream>
#include <exception>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
	Utility::set_digits();
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    const double lat0 = -6.36248605092425, lon0 = 106.82503312826157; // current location of boat
    LocalCartesian proj(lat0, lon0, 100, earth);
      double lat, lon, h; // target location
      double x, y, z;
      x = -20;
      y = 0;
      z = 0;
      
      // Sample reverse calculation
      proj.Reverse(x, y, z, lat, lon, h);
      
      cout << Utility::str(false ? lon : lat, 6 + 5) << " ";

      cout << Utility::str(true ? lon : lat, 6 + 5) << " ";
      
      cout << Utility::str(h, 5) << endl;
    
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
