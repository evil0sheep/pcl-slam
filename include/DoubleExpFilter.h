#ifndef _DoubleExpFilter_h_
#define _DoubleExpFilter_h_

#include <deque>

#include <glm/vec3.hpp>

class DoubleExpFilter {
public:
	glm::dvec3 updatePosition(glm::dvec3 sensedPosition);
	glm::dvec3 getCurrentPosition();

	DoubleExpFilter(glm::dvec3 initialPosition, double dataSmootingFactor, double trendSmoothingFactor);

private:
	double kAlpha;
	double kBeta;
	glm::dvec3 kSmoothedValue;
	glm::dvec3 kTrendValue;
	bool kFirstUpdate;
};

#endif