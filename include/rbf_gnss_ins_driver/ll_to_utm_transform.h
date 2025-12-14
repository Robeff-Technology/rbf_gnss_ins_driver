//
// Created by elaydin on 12.07.2023.
//

#ifndef LL_TO_UTM_TRANSFORM_H
#define LL_TO_UTM_TRANSFORM_H

namespace rbf_gnss_ins_driver
{

class LlToUtmTransform
{
private:
  struct Origin
  {
    int zone;
    double northing;
    double easting;
    double altitude;
  };

public:
  LlToUtmTransform(double lat, double lon, double altitude) { initUTM(lat, lon, altitude); };
  LlToUtmTransform() = delete;
  ~LlToUtmTransform() = default;
  void transform(double lat, double lon, double altitude, double & x, double & y, double & z) const;

  void initUTM(double Lat, double Long, double altitude);

  char UTMLetterDesignator(double Lat);

  void LLtoUTM(
    double Lat, double Long, int zoneNumber, double & UTMNorthing, double & UTMEasting) const;

  int find_zone(double Lat, double long_temp) const;

  Origin m_utm0_;
};

}  // namespace rbf_gnss_ins_driver
#endif
