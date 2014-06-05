#include "../include/DoubleExpFilter.h"

glm::dvec3 DoubleExpFilter::updatePosition(glm::dvec3 sensedPosition) {
	glm::dvec3 newSmoothed;
	glm::dvec3 newTrend;

	if (kFirstUpdate) {
		newSmoothed = sensedPosition;
		newTrend = sensedPosition - kSmoothedValue;
		kFirstUpdate = false;
	} else {
		newSmoothed = kAlpha * sensedPosition + (1.0 - kAlpha) * (kSmoothedValue + kTrendValue);
		newTrend = kBeta * (newSmoothed - kSmoothedValue) + (1.0 - kBeta) * kTrendValue;
	}

	kSmoothedValue = newSmoothed;
	kTrendValue = newTrend;

	return newSmoothed;
}

glm::dvec3 DoubleExpFilter::getCurrentPosition() {
	return kSmoothedValue;
}

DoubleExpFilter::DoubleExpFilter(glm::dvec3 initialPosition, double dataSmootingFactor, double trendSmoothingFactor) {
	kAlpha = dataSmootingFactor;
	kBeta = trendSmoothingFactor;
	kSmoothedValue = initialPosition;
	kFirstUpdate = true;
}